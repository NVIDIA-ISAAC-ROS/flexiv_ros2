/**
 * @file flexiv_hardware_interface.cpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/rrbot_system_multi_interface.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <bit>
#include <tuple>
#include <vector>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/rdk/robot.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"
#include "flexiv_hardware/fault_recovery.hpp"

namespace {

constexpr double kMaxJointVelocity = 2.0;
constexpr double kMaxJointAcceleration = 3.0;

// Bounded wait for the robot to become operational during activation. Brake release dominates the
// duration; a robot that is not ready within this window needs operator attention.
constexpr std::chrono::seconds kActivationOperationalTimeout {30};
constexpr std::chrono::milliseconds kOperationalPollPeriod {200};

// Command/state interface names for Cartesian motion-force control, exported under the
// "<prefix>tcp" interface prefix.
constexpr const char* kCartesianPoseInterfaces[]
    = {"cartesian_pose_x", "cartesian_pose_y", "cartesian_pose_z", "cartesian_pose_qw",
        "cartesian_pose_qx", "cartesian_pose_qy", "cartesian_pose_qz"};
constexpr const char* kCartesianWrenchInterfaces[] = {"cartesian_wrench_fx", "cartesian_wrench_fy",
    "cartesian_wrench_fz", "cartesian_wrench_mx", "cartesian_wrench_my", "cartesian_wrench_mz"};

// Mode names used in start_modes_ for the Cartesian interfaces, which have no
// hardware_interface::HW_IF_* equivalent.
constexpr const char* kStartModeCartesianPose = "cartesian_pose";
constexpr const char* kStartModeCartesianWrench = "cartesian_wrench";

hardware_interface::InterfaceDescription CartesianInterfaceDescription(
    const std::string& tcp_name, const std::string& interface_name)
{
    hardware_interface::InterfaceInfo info;
    info.name = interface_name;
    info.data_type = "double";
    info.enable_limits = false;
    return hardware_interface::InterfaceDescription(tcp_name, info);
}

}

namespace flexiv_hardware {

hardware_interface::CallbackReturn FlexivHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (hardware_interface::SystemInterface::on_init(params)
        != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_positions_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_velocities_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_joint_efforts_.resize(
        info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_gpio_in_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    hw_commands_gpio_out_.resize(flexiv::rdk::kIOPorts, std::numeric_limits<double>::quiet_NaN());
    stop_modes_ = {StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
        StoppingInterface::NONE};
    start_modes_ = {};
    position_controller_running_ = false;
    velocity_controller_running_ = false;
    torque_controller_running_ = false;
    cartesian_motion_controller_running_ = false;
    cartesian_mode_active_ = false;
    runtime_cartesian_switching_ = false;
    controllers_initialized_ = false;

    // A NaN pose command means "no Cartesian setpoint yet", which write() holds the initial TCP
    // pose against. The wrench defaults to zero, i.e. pure motion control.
    hw_commands_cartesian_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    hw_commands_cartesian_wrench_.fill(0.0);
    hw_commands_cartesian_velocity_.fill(0.0);
    hw_commands_cartesian_acceleration_.fill(0.0);
    hw_states_cartesian_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    init_tcp_pose_.fill(0.0);

    if (info_.joints.size() < 7) {
        RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected at least 7.", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Get prefix for joint mapping
    std::string prefix;
    try {
        prefix = info_.hardware_parameters.at("prefix");
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'prefix' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Build RDK to ROS joint mapping
    std::vector<size_t> arm_indices;
    std::vector<size_t> ext_indices;

    // Find 7 arm joints in standard order
    for (int j = 1; j <= 7; ++j) {
        std::string arm_joint_name = prefix + "joint" + std::to_string(j);
        bool found = false;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            if (info_.joints[i].name == arm_joint_name) {
                arm_indices.push_back(i);
                found = true;
                break;
            }
        }
        if (!found) {
            RCLCPP_FATAL(getLogger(), "Could not find arm joint '%s'", arm_joint_name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // Find external axis joints (any joint that is not an arm joint)
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        bool is_arm = false;
        for (size_t arm_idx : arm_indices) {
            if (i == arm_idx) {
                is_arm = true;
                break;
            }
        }
        if (!is_arm) {
            ext_indices.push_back(i);
        }
    }

    // Construct map: external joints first, then arm joints (RDK order)
    rdk_to_ros_map_.clear();
    rdk_to_ros_map_.insert(rdk_to_ros_map_.end(), ext_indices.begin(), ext_indices.end());
    rdk_to_ros_map_.insert(rdk_to_ros_map_.end(), arm_indices.begin(), arm_indices.end());

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        if (joint.command_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' command interface. Expected '%s'",
                joint.name.c_str(), joint.command_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld state interfaces found. 3 expected.",
                joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has '%s' state interface. Expected '%s'",
                joint.name.c_str(), joint.state_interfaces[2].name.c_str(),
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    try {
        info_.hardware_parameters.at("robot_sn");
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'robot_sn' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        auto rdk_control_mode_str = info_.hardware_parameters.at("rdk_control_mode");
        if (rdk_control_mode_str == "joint_position") {
            rdk_control_mode_ = flexiv::rdk::Mode::NRT_JOINT_POSITION;
        } else if (rdk_control_mode_str == "joint_impedance") {
            rdk_control_mode_ = flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE;
        } else if (rdk_control_mode_str == "cartesian_motion_force") {
            rdk_control_mode_ = flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE;
        } else {
            RCLCPP_FATAL(getLogger(),
                "Parameter 'rdk_control_mode' has invalid value '%s'. Options: joint_position, "
                "joint_impedance, cartesian_motion_force",
                rdk_control_mode_str.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    } catch (const std::out_of_range& ex) {
        RCLCPP_FATAL(getLogger(), "Parameter 'rdk_control_mode' not set");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Optional: come up in a joint mode but stay ready to hand the robot over to the Cartesian
    // controller at runtime, for workflows that plan a joint trajectory and then switch to a
    // task-space policy without restarting the driver.
    if (info_.hardware_parameters.count("runtime_cartesian_switching") > 0) {
        const auto& value = info_.hardware_parameters.at("runtime_cartesian_switching");
        runtime_cartesian_switching_ = (value == "true" || value == "True" || value == "1");
    }
    if (runtime_cartesian_switching_
        && rdk_control_mode_ == flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
        // Already Cartesian for the whole session; there is nothing to switch into later.
        runtime_cartesian_switching_ = false;
    }

    // The connection is established in on_configure, which is the lifecycle stage that owns
    // communication with the hardware. This is what lets a lost connection be recovered by
    // cleaning up and reconfiguring, instead of restarting the whole process.
    driver_status_ = std::make_shared<DriverStatus>();
    executor_ = params.executor;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    const std::string robot_sn = info_.hardware_parameters.at("robot_sn");

    try {
        RCLCPP_INFO(getLogger(), "Connecting to robot %s ...", robot_sn.c_str());
        robot_ = std::make_unique<flexiv::rdk::Robot>(robot_sn);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not connect to robot");
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Successfully connected to robot");

    // Check the DoF of the robot against the URDF before exposing any interface.
    if (robot_->info().DoF != info_.joints.size()) {
        RCLCPP_FATAL(getLogger(), "Robot has %ld DoF. Expected %ld (from URDF).",
            robot_->info().DoF, info_.joints.size());
        Disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }

    robot_system_control_ = std::make_unique<SingleRobotSystemControl>(*robot_);
    driver_status_->driver_state.store(DriverState::FAULT);

    // Host the recovery interface on the controller manager's executor, so that all blocking
    // system control calls happen off the real-time control loop without needing a thread here.
    auto executor = executor_.lock();
    if (!executor) {
        RCLCPP_FATAL(getLogger(),
            "No executor available to host the recovery interface. The controller manager must "
            "provide one through HardwareComponentInterfaceParams.");
        Disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }
    recovery_node_
        = std::make_shared<RecoveryNode>(robot_sn, *robot_system_control_, driver_status_);
    executor->add_node(recovery_node_->get_node_base_interface());

    // The impedance setters only apply to the joint impedance control modes, but the interface is
    // advertised either way so that a request made against a joint_position driver is answered with
    // an explanation instead of a missing service.
    std::vector<std::string> joint_names;
    joint_names.reserve(info_.joints.size());
    for (const auto& joint : info_.joints) {
        joint_names.push_back(joint.name);
    }

    JointImpedanceBounds bounds;
    bounds.k_q_nom = ConvertRDKToROSOrder(robot_->info().K_q_nom, rdk_to_ros_map_);
    bounds.tau_max = ConvertRDKToROSOrder(robot_->info().tau_max, rdk_to_ros_map_);

    // The node works in ROS joint order and knows nothing about the RDK; these three closures are
    // where the order is translated and the RDK is actually called.
    JointImpedanceSetters setters;
    setters.set_joint_impedance
        = [this](const std::vector<double>& k_q, const std::vector<double>& z_q) {
              robot_->SetJointImpedance(ConvertROSToRDKOrder(k_q, rdk_to_ros_map_),
                  ConvertROSToRDKOrder(z_q, rdk_to_ros_map_));
          };
    setters.set_max_contact_torque = [this](const std::vector<double>& max_torques) {
        robot_->SetMaxContactTorque(ConvertROSToRDKOrder(max_torques, rdk_to_ros_map_));
    };
    setters.set_joint_inertia_scale = [this](const std::vector<double>& inertia_scales) {
        robot_->SetJointInertiaScale(ConvertROSToRDKOrder(inertia_scales, rdk_to_ros_map_));
    };

    joint_impedance_config_node_
        = std::make_shared<JointImpedanceConfigNode>(robot_sn, std::move(joint_names),
            std::move(bounds), rdk_control_mode_ == flexiv::rdk::Mode::NRT_JOINT_IMPEDANCE,
            driver_status_, std::move(setters));
    executor->add_node(joint_impedance_config_node_->get_node_base_interface());

    if (!ResolveInterfaceHandles()) {
        Disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

void FlexivHardwareInterface::TrackPositionChangeAcrossInterruption()
{
    const bool ready = driver_status_->driver_state.load() == DriverState::READY;

    if (was_ready_ && !ready) {
        // Joint states stop being refreshed once the robot is no longer operational, so this still
        // holds the last position the robot was known to be at before it stopped.
        positions_before_interruption_ = hw_states_joint_positions_;
    } else if (!was_ready_ && ready && !positions_before_interruption_.empty()) {
        if (driver_status_->RequiresControllerRestart()) {
            const double deviation
                = MaxJointDeviation(positions_before_interruption_, hw_states_joint_positions_);

            RCLCPP_WARN(getLogger(),
                "The robot is ready again, %.3f rad from the last commanded "
                "position. Motion stays withheld until the controllers are restarted.",
                deviation);
        }
        positions_before_interruption_.clear();
    }

    was_ready_ = ready;
}

void FlexivHardwareInterface::StopIfOperational()
{
    if (robot_ && robot_->connected() && robot_->operational()) {
        robot_->Stop();
    }
}

void FlexivHardwareInterface::Disconnect()
{
    if (recovery_node_) {
        if (auto executor = executor_.lock()) {
            executor->remove_node(recovery_node_->get_node_base_interface());
        }
        recovery_node_.reset();
    }
    // Torn down before robot_ below, since its closures capture this and call through it.
    if (joint_impedance_config_node_) {
        if (auto executor = executor_.lock()) {
            executor->remove_node(joint_impedance_config_node_->get_node_base_interface());
        }
        joint_impedance_config_node_.reset();
    }
    StopCartesianConfigServices();
    robot_system_control_.reset();
    robot_.reset();
    if (driver_status_) {
        driver_status_->driver_state.store(DriverState::UNINITIALIZED);
    }
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Cleaning up, closing the connection to the robot ...");
    Disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    Disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_ERROR(getLogger(), "Hardware component entered the error state, stopping the robot");

    try {
        StopIfOperational();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Could not stop the robot: %s", e.what());
    }

    Disconnect();

    // Returning SUCCESS puts the component in UNCONFIGURED, from which it can be configured and
    // activated again. Returning FAILURE or ERROR here would finalize it, and the whole process
    // would have to be restarted to recover.
    return hardware_interface::CallbackReturn::SUCCESS;
}

rclcpp::Logger FlexivHardwareInterface::getLogger()
{
    return rclcpp::get_logger("FlexivHardwareInterface");
}

std::vector<hardware_interface::InterfaceDescription>
FlexivHardwareInterface::export_unlisted_state_interface_descriptions()
{
    hardware_interface::InterfaceInfo states_info;
    states_info.name = "flexiv_robot_states";
    states_info.data_type = "double";
    states_info.enable_limits = false;

    std::vector<hardware_interface::InterfaceDescription> descriptions {
        hardware_interface::InterfaceDescription(
            info_.hardware_parameters.at("robot_sn"), states_info)};

    // The measured TCP pose is exported for every driver, whatever control mode it runs in, so
    // that a task-space consumer can read it without claiming a command interface.
    const std::string tcp_name = info_.hardware_parameters.at("prefix") + "tcp";
    for (const auto& interface_name : kCartesianPoseInterfaces) {
        descriptions.push_back(CartesianInterfaceDescription(tcp_name, interface_name));
    }

    return descriptions;
}

std::vector<hardware_interface::InterfaceDescription>
FlexivHardwareInterface::export_unlisted_command_interface_descriptions()
{
    // Declared here rather than in the URDF, because they belong to the hardware rather than to
    // any joint, and because a description that never mentions them still has to work.
    std::vector<hardware_interface::InterfaceDescription> descriptions;

    const std::string tcp_name = info_.hardware_parameters.at("prefix") + "tcp";
    for (const auto& interface_name : kCartesianPoseInterfaces) {
        descriptions.push_back(CartesianInterfaceDescription(tcp_name, interface_name));
    }
    for (const auto& interface_name : kCartesianWrenchInterfaces) {
        descriptions.push_back(CartesianInterfaceDescription(tcp_name, interface_name));
    }

    return descriptions;
}

bool FlexivHardwareInterface::ResolveInterfaceHandles()
{
    handles_state_joint_positions_.clear();
    handles_state_joint_velocities_.clear();
    handles_state_joint_efforts_.clear();
    handles_command_joint_positions_.clear();
    handles_command_joint_velocities_.clear();
    handles_command_joint_efforts_.clear();
    handles_state_gpio_in_.clear();
    handles_command_gpio_out_.clear();
    handles_state_cartesian_pose_.clear();
    handles_command_cartesian_pose_.clear();
    handles_command_cartesian_wrench_.clear();

    handles_state_joint_positions_.reserve(info_.joints.size());
    handles_state_joint_velocities_.reserve(info_.joints.size());
    handles_state_joint_efforts_.reserve(info_.joints.size());
    handles_command_joint_positions_.reserve(info_.joints.size());
    handles_command_joint_velocities_.reserve(info_.joints.size());
    handles_command_joint_efforts_.reserve(info_.joints.size());

    try {
        // Resolved from info_.joints rather than from the framework's unordered maps, so that the
        // handle order matches the joint order the RDK index map was built against.
        for (const auto& joint : info_.joints) {
            handles_state_joint_positions_.push_back(
                get_state_interface_handle(joint.name + "/" + hardware_interface::HW_IF_POSITION));
            handles_state_joint_velocities_.push_back(
                get_state_interface_handle(joint.name + "/" + hardware_interface::HW_IF_VELOCITY));
            handles_state_joint_efforts_.push_back(
                get_state_interface_handle(joint.name + "/" + hardware_interface::HW_IF_EFFORT));
            handles_command_joint_positions_.push_back(get_command_interface_handle(
                joint.name + "/" + hardware_interface::HW_IF_POSITION));
            handles_command_joint_velocities_.push_back(get_command_interface_handle(
                joint.name + "/" + hardware_interface::HW_IF_VELOCITY));
            handles_command_joint_efforts_.push_back(
                get_command_interface_handle(joint.name + "/" + hardware_interface::HW_IF_EFFORT));
        }

        const std::string prefix = info_.hardware_parameters.at("prefix");
        handles_state_gpio_in_.reserve(hw_states_gpio_in_.size());
        handles_command_gpio_out_.reserve(hw_commands_gpio_out_.size());
        for (std::size_t i = 0; i < hw_states_gpio_in_.size(); i++) {
            handles_state_gpio_in_.push_back(
                get_state_interface_handle(prefix + "gpio/digital_input_" + std::to_string(i)));
        }
        for (std::size_t i = 0; i < hw_commands_gpio_out_.size(); i++) {
            handles_command_gpio_out_.push_back(
                get_command_interface_handle(prefix + "gpio/digital_output_" + std::to_string(i)));
        }

        handle_state_flexiv_robot_states_ = get_state_interface_handle(
            info_.hardware_parameters.at("robot_sn") + "/flexiv_robot_states");

        const std::string tcp_name = prefix + "tcp";
        for (const auto& interface_name : kCartesianPoseInterfaces) {
            handles_state_cartesian_pose_.push_back(
                get_state_interface_handle(tcp_name + "/" + interface_name));
            handles_command_cartesian_pose_.push_back(
                get_command_interface_handle(tcp_name + "/" + interface_name));
        }
        for (const auto& interface_name : kCartesianWrenchInterfaces) {
            handles_command_cartesian_wrench_.push_back(
                get_command_interface_handle(tcp_name + "/" + interface_name));
        }
    } catch (const std::exception& ex) {
        RCLCPP_FATAL(getLogger(), "Could not resolve interface handles: %s", ex.what());
        return false;
    }

    // The states pointer never moves, so it is published once here rather than every read().
    std::ignore = set_state(
        handle_state_flexiv_robot_states_, std::bit_cast<double>(&hw_flexiv_robot_states_), true);

    return true;
}

void FlexivHardwareInterface::PublishStatesToInterfaces()
{
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        std::ignore = set_state(handles_state_joint_positions_[i], hw_states_joint_positions_[i],
            /*wait_until_set=*/false);
        std::ignore = set_state(handles_state_joint_velocities_[i], hw_states_joint_velocities_[i],
            /*wait_until_set=*/false);
        std::ignore = set_state(
            handles_state_joint_efforts_[i], hw_states_joint_efforts_[i], /*wait_until_set=*/false);
    }
    for (std::size_t i = 0; i < handles_state_gpio_in_.size(); i++) {
        std::ignore
            = set_state(handles_state_gpio_in_[i], hw_states_gpio_in_[i], /*wait_until_set=*/false);
    }
    for (std::size_t i = 0; i < handles_state_cartesian_pose_.size(); i++) {
        std::ignore = set_state(handles_state_cartesian_pose_[i], hw_states_cartesian_pose_[i],
            /*wait_until_set=*/false);
    }
}

void FlexivHardwareInterface::PushCommandsToInterfaces()
{
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        std::ignore = set_command(handles_command_joint_positions_[i],
            hw_commands_joint_positions_[i], /*wait_until_set=*/true);
        std::ignore = set_command(handles_command_joint_velocities_[i],
            hw_commands_joint_velocities_[i], /*wait_until_set=*/true);
        std::ignore = set_command(handles_command_joint_efforts_[i], hw_commands_joint_efforts_[i],
            /*wait_until_set=*/true);
    }
    for (std::size_t i = 0; i < handles_command_cartesian_pose_.size(); i++) {
        std::ignore = set_command(handles_command_cartesian_pose_[i],
            hw_commands_cartesian_pose_[i], /*wait_until_set=*/true);
    }
    for (std::size_t i = 0; i < handles_command_cartesian_wrench_.size(); i++) {
        std::ignore = set_command(handles_command_cartesian_wrench_[i],
            hw_commands_cartesian_wrench_[i], /*wait_until_set=*/true);
    }
}

void FlexivHardwareInterface::ReadCommandsFromInterfaces()
{
    // A handle whose value cannot be read without blocking keeps its previous buffered value, which
    // for position commands is the last setpoint and for efforts is NaN -- both of which write()
    // already handles.
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        std::ignore = get_command(handles_command_joint_positions_[i],
            hw_commands_joint_positions_[i], /*wait_until_get=*/false);
        std::ignore = get_command(handles_command_joint_velocities_[i],
            hw_commands_joint_velocities_[i], /*wait_until_get=*/false);
        std::ignore = get_command(handles_command_joint_efforts_[i], hw_commands_joint_efforts_[i],
            /*wait_until_get=*/false);
    }
    for (std::size_t i = 0; i < handles_command_gpio_out_.size(); i++) {
        std::ignore = get_command(
            handles_command_gpio_out_[i], hw_commands_gpio_out_[i], /*wait_until_get=*/false);
    }
    for (std::size_t i = 0; i < handles_command_cartesian_pose_.size(); i++) {
        std::ignore = get_command(handles_command_cartesian_pose_[i],
            hw_commands_cartesian_pose_[i], /*wait_until_get=*/false);
    }
    for (std::size_t i = 0; i < handles_command_cartesian_wrench_.size(); i++) {
        std::ignore = get_command(handles_command_cartesian_wrench_[i],
            hw_commands_cartesian_wrench_[i], /*wait_until_get=*/false);
    }
}

bool FlexivHardwareInterface::WaitUntilOperational(std::chrono::seconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto next_log = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() < deadline) {
        if (robot_->operational()) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= next_log) {
            RCLCPP_INFO(getLogger(), "Waiting for the robot to become operational: %s",
                OperationalStatusName(robot_->operational_status()).c_str());
            next_log = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        }
        std::this_thread::sleep_for(kOperationalPollPeriod);
    }
    return robot_->operational();
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Starting... please wait...");

    try {
        // Report the condition through the classifier, so that a pressed E-stop or a robot left in
        // Manual mode names the operator action instead of surfacing as an RDK exception.
        const auto condition = robot_system_control_->condition();
        const auto policy = ClassifyRecoveryPolicy(condition);
        if (policy != RecoveryPolicy::NONE) {
            RCLCPP_WARN(getLogger(), "%s", DescribeRobotCondition(condition).c_str());
        }
        if (policy == RecoveryPolicy::SAFETY_LOCKOUT || policy == RecoveryPolicy::WAIT_OPERATOR) {
            RCLCPP_FATAL(
                getLogger(), "Cannot start: %s", DescribeRobotCondition(condition).c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Clear fault on robot server if any
        if (robot_->fault()) {
            RCLCPP_WARN(getLogger(), "Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            if (!robot_->ClearFault()) {
                RCLCPP_FATAL(getLogger(), "Fault cannot be cleared, exiting ...");
                return hardware_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(getLogger(), "Fault on robot server is cleared");
        }

        // Enable the robot
        RCLCPP_INFO(getLogger(), "Enabling robot ...");
        robot_->Enable();

        // Wait for the robot to become operational, bounded so that a robot that never becomes
        // ready fails the activation instead of hanging the controller manager forever.
        if (!WaitUntilOperational(kActivationOperationalTimeout)) {
            RCLCPP_FATAL(getLogger(), "Robot did not become operational within %ld s. %s",
                static_cast<long>(kActivationOperationalTimeout.count()),
                DescribeRobotCondition(robot_system_control_->condition()).c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(getLogger(), "Robot is now operational");

        // Unlock external axes if any
        if (robot_->info().DoF_e > 0) {
            robot_->LockExternalAxes(false);
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not enable robot.");
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Cartesian control is prepared here, at activation, before the 1 kHz loop is running. Zeroing
    // the force/torque sensor executes a primitive that takes seconds, so it must never run from
    // perform_command_mode_switch().
    if (rdk_control_mode_ == flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE
        || runtime_cartesian_switching_) {
        if (!PrepareCartesianControl()) {
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // The robot is enabled but in IDLE: a controller start has to establish the control mode and
    // synchronize the command buffers before any motion may be streamed.
    driver_status_->commands_synchronized.store(false);
    driver_status_->driver_state.store(DriverState::READY);

    RCLCPP_INFO(getLogger(), "System successfully started!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

bool FlexivHardwareInterface::ZeroFTSensor(std::string& error)
{
    // Other RDK clients contend for the robot at activation -- flexiv_gripper's Gripper::Init takes
    // about ten seconds and makes SwitchMode fail while it is in flight -- so retry instead of
    // giving up on the first refusal.
    for (int attempt = 1; attempt <= kZeroFTSensorMaxAttempts; ++attempt) {
        try {
            robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
            robot_->ExecutePrimitive(
                "ZeroFTSensor", std::map<std::string, flexiv::rdk::FlexivDataTypes> {});

            // Wait for the primitive to finish, bounded so that a primitive which never terminates
            // cannot hold the executor thread indefinitely.
            int waited_ms = 0;
            while (!std::get<int>(robot_->primitive_states()["terminated"])) {
                if (waited_ms >= kZeroFTSensorPrimitiveTimeoutMs) {
                    throw std::runtime_error("ZeroFTSensor primitive did not terminate");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                waited_ms += 100;
            }
            RCLCPP_INFO(getLogger(), "Sensor zeroing complete");
            return true;
        } catch (const std::exception& e) {
            error = e.what();
            RCLCPP_WARN(getLogger(), "Force/torque sensor zeroing attempt %d of %d failed: %s",
                attempt, kZeroFTSensorMaxAttempts, error.c_str());
            if (attempt < kZeroFTSensorMaxAttempts) {
                std::this_thread::sleep_for(std::chrono::milliseconds(kZeroFTSensorRetryDelayMs));
            }
        }
    }
    return false;
}

bool FlexivHardwareInterface::PrepareCartesianControl()
{
    // Zeroing is only valid while the robot is parked and empty-handed, which is why it happens
    // once here rather than at every mode switch.
    RCLCPP_WARN(getLogger(),
        "Zeroing force/torque sensor. Make sure nothing is in contact with the robot.");

    const bool cartesian_from_start
        = (rdk_control_mode_ == flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

    std::string zeroing_error;
    if (!ZeroFTSensor(zeroing_error)) {
        if (cartesian_from_start) {
            // Cartesian is the only mode this driver has; there is nothing to fall back to.
            RCLCPP_FATAL(
                getLogger(), "Failed to zero force/torque sensor: %s", zeroing_error.c_str());
            return false;
        }
        // Joint control is unaffected, so come up anyway and only give up the Cartesian handover.
        // Clearing the flag makes perform_command_mode_switch() refuse the switch later rather
        // than command Cartesian against a sensor that was never zeroed.
        RCLCPP_ERROR(getLogger(),
            "Failed to zero force/torque sensor: %s. Runtime Cartesian switching is disabled; the "
            "driver continues in its joint mode.",
            zeroing_error.c_str());
        runtime_cartesian_switching_ = false;
        try {
            robot_->SwitchMode(flexiv::rdk::Mode::IDLE);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(getLogger(), "Failed to return the robot to IDLE: %s", e.what());
            return false;
        }
        return true;
    }

    try {
        init_tcp_pose_ = robot_->states().tcp_pose;
        if (cartesian_from_start) {
            robot_->SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
            robot_->SetForceControlAxis(
                std::array<bool, kCartDoF> {false, false, false, false, false, false});
            cartesian_mode_active_ = true;
            RCLCPP_INFO(getLogger(), "Switched to RT_CARTESIAN_MOTION_FORCE mode");
        } else {
            // Leave the robot IDLE, which is how activation ends when no zeroing runs. Other RDK
            // clients need it: flexiv_gripper's Tool::Switch fails with "Robot is not in IDLE mode"
            // against any control mode. perform_command_mode_switch() sets the control mode when
            // the first controller is activated.
            robot_->SwitchMode(flexiv::rdk::Mode::IDLE);
            RCLCPP_INFO(getLogger(),
                "Cartesian mode prepared; runtime switching to RT_CARTESIAN_MOTION_FORCE is "
                "enabled");
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not prepare Cartesian control: %s", e.what());
        return false;
    }

    RCLCPP_INFO(getLogger(), "Initial TCP pose: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        init_tcp_pose_[0], init_tcp_pose_[1], init_tcp_pose_[2], init_tcp_pose_[3],
        init_tcp_pose_[4], init_tcp_pose_[5], init_tcp_pose_[6]);

    StartCartesianConfigServices();

    return true;
}

void FlexivHardwareInterface::StartCartesianConfigServices()
{
    auto executor = executor_.lock();
    if (!executor) {
        RCLCPP_ERROR(getLogger(),
            "No executor available to host the Cartesian configuration services. Cartesian control "
            "runs, but its impedance and force-control settings cannot be changed at runtime.");
        return;
    }

    std::string node_name = info_.hardware_parameters.at("prefix");
    std::replace(node_name.begin(), node_name.end(), '-', '_');
    node_name += "flexiv_cartesian_config";

    cartesian_config_node_ = rclcpp::Node::make_shared(node_name);

    set_cartesian_impedance_srv_
        = cartesian_config_node_->create_service<flexiv_msgs::srv::SetCartesianImpedance>(
            "~/set_cartesian_impedance",
            std::bind(&FlexivHardwareInterface::SetCartesianImpedanceCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    set_null_space_posture_srv_
        = cartesian_config_node_->create_service<flexiv_msgs::srv::SetNullSpacePosture>(
            "~/set_null_space_posture",
            std::bind(&FlexivHardwareInterface::SetNullSpacePostureCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    set_max_contact_wrench_srv_
        = cartesian_config_node_->create_service<flexiv_msgs::srv::SetMaxContactWrench>(
            "~/set_max_contact_wrench",
            std::bind(&FlexivHardwareInterface::SetMaxContactWrenchCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    set_force_control_frame_srv_
        = cartesian_config_node_->create_service<flexiv_msgs::srv::SetForceControlFrame>(
            "~/set_force_control_frame",
            std::bind(&FlexivHardwareInterface::SetForceControlFrameCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    set_force_control_axis_srv_
        = cartesian_config_node_->create_service<flexiv_msgs::srv::SetForceControlAxis>(
            "~/set_force_control_axis",
            std::bind(&FlexivHardwareInterface::SetForceControlAxisCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    executor->add_node(cartesian_config_node_->get_node_base_interface());

    RCLCPP_INFO(getLogger(), "Cartesian configuration services available under /%s",
        cartesian_config_node_->get_name());
}

void FlexivHardwareInterface::StopCartesianConfigServices()
{
    if (!cartesian_config_node_) {
        return;
    }
    if (auto executor = executor_.lock()) {
        executor->remove_node(cartesian_config_node_->get_node_base_interface());
    }
    set_cartesian_impedance_srv_.reset();
    set_null_space_posture_srv_.reset();
    set_max_contact_wrench_srv_.reset();
    set_force_control_frame_srv_.reset();
    set_force_control_axis_srv_.reset();
    cartesian_config_node_.reset();
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Stopping... please wait...");

    // Hold off write() before stopping, so the real-time loop cannot stream a command into a robot
    // that is being brought to a halt.
    driver_status_->driver_state.store(DriverState::FAULT);

    StopCartesianConfigServices();
    cartesian_mode_active_ = false;
    cartesian_motion_controller_running_ = false;

    try {
        StopIfOperational();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Could not stop the robot: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "System successfully stopped!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FlexivHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Latch the robot condition for the recovery node. These are all non-blocking accessors over
    // cached state, so they are safe to poll from the real-time loop.
    driver_status_->Latch(*robot_system_control_);

    // Recovery owns the driver state while it runs, so this is a no-op for its duration.
    driver_status_->TryApplyDerivedDriverState();

    // A lost connection is the only condition the driver cannot report or recover from in place,
    // so it is the only one escalated to the controller manager. Every other fault keeps the
    // component ACTIVE, which keeps the status topic and the recovery action reachable.
    if (!driver_status_->connected.load()) {
        RCLCPP_ERROR(getLogger(), "Lost connection with the robot");
        return hardware_interface::return_type::ERROR;
    }

    if (driver_status_->operational.load()) {
        hw_flexiv_robot_states_ = robot_->states();

        // Read joint states
        // Map RDK states (RDK order) to Hardware Interface states (ROS order)
        for (size_t rdk_idx = 0; rdk_idx < robot_->info().DoF; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            if (ros_idx < info_.joints.size()) {
                hw_states_joint_positions_[ros_idx] = hw_flexiv_robot_states_.q[rdk_idx];
                hw_states_joint_velocities_[ros_idx] = hw_flexiv_robot_states_.dtheta[rdk_idx];
                hw_states_joint_efforts_[ros_idx] = hw_flexiv_robot_states_.tau[rdk_idx];
            }
        }

        // Read the measured TCP pose, in the RDK's [x, y, z, qw, qx, qy, qz] order
        hw_states_cartesian_pose_ = hw_flexiv_robot_states_.tcp_pose;

        // Read GPIO input states
        auto gpio_in = robot_->digital_inputs();
        for (size_t i = 0; i < hw_states_gpio_in_.size(); i++) {
            hw_states_gpio_in_[i] = static_cast<double>(gpio_in[i]);
        }
    }

    TrackPositionChangeAcrossInterruption();

    PublishStatesToInterfaces();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Issue no RDK call unless the robot is ready. While recovery runs it changes the control mode
    // and the fault state, and this early return is what guarantees the real-time loop is
    // quiescent for the duration without needing a lock on the hot path.
    if (driver_status_->driver_state.load() != DriverState::READY) {
        return hardware_interface::return_type::OK;
    }

    ReadCommandsFromInterfaces();

    // Initialize target vectors to hold position
    std::vector<double> target_pos(robot_->info().DoF);
    std::vector<double> target_vel(robot_->info().DoF);

    std::vector<double> max_vel(robot_->info().DoF, kMaxJointVelocity);
    std::vector<double> max_acc(robot_->info().DoF, kMaxJointAcceleration);

    bool is_pos_nan = false;
    bool is_vel_nan = false;
    bool is_eff_nan = false;
    for (std::size_t i = 0; i < robot_->info().DoF; i++) {
        if (hw_commands_joint_positions_[i] != hw_commands_joint_positions_[i]) {
            is_pos_nan = true;
        }
        if (hw_commands_joint_velocities_[i] != hw_commands_joint_velocities_[i]) {
            is_vel_nan = true;
        }
        if (hw_commands_joint_efforts_[i] != hw_commands_joint_efforts_[i]) {
            is_eff_nan = true;
        }
    }

    // Withhold motion until a controller restart has re-synchronized the command buffers.
    const bool stream_motion = driver_status_->commands_synchronized.load();

    if (stream_motion && position_controller_running_ && robot_->mode() == rdk_control_mode_
        && !is_pos_nan) {
        // Map ROS commands to RDK targets
        for (size_t rdk_idx = 0; rdk_idx < robot_->info().DoF; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            target_pos[rdk_idx] = hw_commands_joint_positions_[ros_idx];
        }
        robot_->SendJointPosition(target_pos, target_vel, max_vel, max_acc);
    } else if (stream_motion && velocity_controller_running_ && robot_->mode() == rdk_control_mode_
               && !is_vel_nan) {
        // Map ROS commands/states to RDK targets
        for (size_t rdk_idx = 0; rdk_idx < robot_->info().DoF; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            target_pos[rdk_idx] = hw_states_joint_positions_[ros_idx];
            target_vel[rdk_idx] = hw_commands_joint_velocities_[ros_idx];
        }
        robot_->SendJointPosition(target_pos, target_vel, max_vel, max_acc);
    } else if (stream_motion && torque_controller_running_
               && robot_->mode() == flexiv::rdk::Mode::RT_JOINT_TORQUE && !is_eff_nan) {
        std::vector<double> target_torque(robot_->info().DoF);
        // Map ROS commands to RDK targets
        for (size_t rdk_idx = 0; rdk_idx < robot_->info().DoF; ++rdk_idx) {
            size_t ros_idx = rdk_to_ros_map_[rdk_idx];
            target_torque[rdk_idx] = hw_commands_joint_efforts_[ros_idx];
        }
        robot_->StreamJointTorque(target_torque, true, true);
    } else if (stream_motion && cartesian_mode_active_
               && robot_->mode() == flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
        if (cartesian_motion_controller_running_ && IsCartesianCommandValid()) {
            robot_->StreamCartesianMotionForce(hw_commands_cartesian_pose_,
                hw_commands_cartesian_wrench_, hw_commands_cartesian_velocity_,
                hw_commands_cartesian_acceleration_);
        } else {
            // RT_CARTESIAN_MOTION_FORCE needs a command every cycle, so hold the pose the mode was
            // entered at until the controller publishes its first setpoint.
            const std::array<double, kCartDoF> zero_wrench {};
            robot_->StreamCartesianMotionForce(init_tcp_pose_, zero_wrench);
        }
    }

    // Write digital output
    std::map<unsigned int, bool> digital_outputs;
    for (size_t i = 0; i < hw_commands_gpio_out_.size(); i++) {
        if (hw_commands_gpio_out_[i] != hw_commands_gpio_out_[i]) {
            continue;
        }
        digital_outputs[i] = static_cast<bool>(hw_commands_gpio_out_[i]);
    }
    // Check if there are changes in the digital output values
    bool digital_outputs_changed = false;
    for (const auto& [index, value] : digital_outputs) {
        if (current_digital_outputs_[index] != value) {
            current_digital_outputs_[index] = value;
            digital_outputs_changed = true;
        }
    }
    current_digital_outputs_.clear();
    for (const auto& [index, value] : digital_outputs) {
        current_digital_outputs_[index] = value;
    }

    // Set digital outputs
    if (digital_outputs_changed && !digital_outputs.empty()) {
        robot_->SetDigitalOutputs(digital_outputs);
    }

    return hardware_interface::return_type::OK;
}

void FlexivHardwareInterface::SynchronizeCommandsWithState()
{
    // Called from perform_command_mode_switch(), which is the controller restart the driver
    // requires after a fault. Once the buffers hold the measured position, motion may stream again.
    driver_status_->commands_synchronized.store(true);
    // Position commands start from where the robot actually is, so the first write() after a mode
    // switch commands a hold instead of whatever setpoint was left over from before.
    hw_commands_joint_positions_ = hw_states_joint_positions_;
    std::fill(hw_commands_joint_velocities_.begin(), hw_commands_joint_velocities_.end(), 0.0);

    // Effort commands are deliberately left as NaN rather than zeroed. write() skips streaming
    // while they are NaN, whereas a zero torque command is streamed with gravity compensation
    // enabled and would leave the arm floating freely instead of holding.
    std::fill(hw_commands_joint_efforts_.begin(), hw_commands_joint_efforts_.end(),
        std::numeric_limits<double>::quiet_NaN());

    // Cartesian pose commands follow the same rule as efforts: NaN means "no setpoint yet", which
    // write() answers by holding init_tcp_pose_ instead of a stale pose from a previous session.
    hw_commands_cartesian_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    hw_commands_cartesian_wrench_.fill(0.0);
    hw_commands_cartesian_velocity_.fill(0.0);
    hw_commands_cartesian_acceleration_.fill(0.0);

    // The command buffers no longer alias the framework's interface storage, so the synchronized
    // values have to be pushed across explicitly. Called off the real-time path, so this waits for
    // the lock.
    PushCommandsToInterfaces();
}

hardware_interface::return_type FlexivHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    start_modes_.clear();
    stop_modes_.clear();

    const std::string tcp_name = info_.hardware_parameters.at("prefix") + "tcp";

    // Starting interfaces
    for (const auto& key : start_interfaces) {
        for (std::size_t i = 0; i < info_.joints.size(); i++) {
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                start_modes_.push_back(hardware_interface::HW_IF_POSITION);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
                start_modes_.push_back(hardware_interface::HW_IF_EFFORT);
            }
        }
        for (const auto& interface_name : kCartesianPoseInterfaces) {
            if (key == tcp_name + "/" + interface_name) {
                start_modes_.push_back(kStartModeCartesianPose);
            }
        }
        for (const auto& interface_name : kCartesianWrenchInterfaces) {
            if (key == tcp_name + "/" + interface_name) {
                start_modes_.push_back(kStartModeCartesianWrench);
            }
        }
    }

    // A Cartesian controller claims all seven pose interfaces, and optionally all six wrench
    // interfaces; a partial claim would leave write() streaming a half-specified target.
    const auto cartesian_pose_count
        = std::count(start_modes_.begin(), start_modes_.end(), kStartModeCartesianPose);
    const auto cartesian_wrench_count
        = std::count(start_modes_.begin(), start_modes_.end(), kStartModeCartesianWrench);
    if (cartesian_pose_count > 0 && static_cast<size_t>(cartesian_pose_count) != kCartPoseSize) {
        RCLCPP_ERROR(getLogger(), "All Cartesian pose interfaces must be claimed together");
        return hardware_interface::return_type::ERROR;
    }
    if (cartesian_wrench_count > 0 && static_cast<size_t>(cartesian_wrench_count) != kCartDoF) {
        RCLCPP_ERROR(getLogger(), "All Cartesian wrench interfaces must be claimed together");
        return hardware_interface::return_type::ERROR;
    }

    const size_t joint_start_count
        = start_modes_.size() - static_cast<size_t>(cartesian_pose_count + cartesian_wrench_count);

    // All joints must be given new command mode at the same time
    if (joint_start_count != 0 && joint_start_count != info_.joints.size()) {
        return hardware_interface::return_type::ERROR;
    }
    // All joints must have the same command mode
    if (joint_start_count != 0 && cartesian_pose_count == 0 && cartesian_wrench_count == 0
        && !std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin())) {
        return hardware_interface::return_type::ERROR;
    }

    // Stop motion on all relevant joints that are stopping
    for (const auto& key : stop_interfaces) {
        for (std::size_t i = 0; i < info_.joints.size(); i++) {
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
                stop_modes_.push_back(StoppingInterface::STOP_POSITION);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
                stop_modes_.push_back(StoppingInterface::STOP_VELOCITY);
            }
            if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
                stop_modes_.push_back(StoppingInterface::STOP_EFFORT);
            }
        }
        for (const auto& interface_name : kCartesianPoseInterfaces) {
            if (key == tcp_name + "/" + interface_name) {
                stop_modes_.push_back(StoppingInterface::STOP_CARTESIAN);
            }
        }
        for (const auto& interface_name : kCartesianWrenchInterfaces) {
            if (key == tcp_name + "/" + interface_name) {
                stop_modes_.push_back(StoppingInterface::STOP_CARTESIAN);
            }
        }
    }

    const auto cartesian_stop_count
        = std::count(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_CARTESIAN);
    const size_t joint_stop_count = stop_modes_.size() - static_cast<size_t>(cartesian_stop_count);

    // stop all interfaces at the same time
    if (joint_stop_count != 0
        && (joint_stop_count != info_.joints.size()
            || (cartesian_stop_count == 0
                && !std::equal(stop_modes_.begin() + 1, stop_modes_.end(), stop_modes_.begin())))) {
        return hardware_interface::return_type::ERROR;
    }

    controllers_initialized_ = true;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FlexivHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    if (stop_modes_.size() != 0
        && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POSITION)
               != stop_modes_.end()) {
        position_controller_running_ = false;
        StopIfOperational();
    } else if (stop_modes_.size() != 0
               && std::find(
                      stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY)
                      != stop_modes_.end()) {
        velocity_controller_running_ = false;
        StopIfOperational();
    } else if (stop_modes_.size() != 0
               && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_EFFORT)
                      != stop_modes_.end()) {
        torque_controller_running_ = false;
        StopIfOperational();
    } else if (stop_modes_.size() != 0
               && std::find(
                      stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_CARTESIAN)
                      != stop_modes_.end()) {
        cartesian_motion_controller_running_ = false;
        // Leaving Cartesian control; gate the Cartesian write() path off again.
        cartesian_mode_active_ = false;
        StopIfOperational();
    }

    if (start_modes_.size() != 0
        && std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION)
               != start_modes_.end()) {
        velocity_controller_running_ = false;
        torque_controller_running_ = false;
        cartesian_motion_controller_running_ = false;
        // Leaving Cartesian control; gate the Cartesian write() path off again.
        cartesian_mode_active_ = false;

        // Hold joints before user commands arrives
        SynchronizeCommandsWithState();

        // Set to joint position or joint impedance mode
        robot_->SwitchMode(rdk_control_mode_);

        // The robot resets its joint impedance properties on mode entry, so whatever was set has to
        // be re-applied before any motion is streamed.
        if (joint_impedance_config_node_ && !joint_impedance_config_node_->Reapply()) {
            RCLCPP_FATAL(getLogger(),
                "Could not re-apply the joint impedance properties. The robot would run at nominal "
                "stiffness instead of the requested one, so the controller start is refused.");
            driver_status_->commands_synchronized.store(false);
            StopIfOperational();
            return hardware_interface::return_type::ERROR;
        }

        position_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(
                      start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_VELOCITY)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        torque_controller_running_ = false;
        cartesian_motion_controller_running_ = false;
        // Leaving Cartesian control; gate the Cartesian write() path off again.
        cartesian_mode_active_ = false;

        // Hold joints before user commands arrives
        SynchronizeCommandsWithState();

        // Set to joint position or joint impedance mode
        robot_->SwitchMode(rdk_control_mode_);

        // The robot resets its joint impedance properties on mode entry, so whatever was set has to
        // be re-applied before any motion is streamed.
        if (joint_impedance_config_node_ && !joint_impedance_config_node_->Reapply()) {
            RCLCPP_FATAL(getLogger(),
                "Could not re-apply the joint impedance properties. The robot would run at nominal "
                "stiffness instead of the requested one, so the controller start is refused.");
            driver_status_->commands_synchronized.store(false);
            StopIfOperational();
            return hardware_interface::return_type::ERROR;
        }

        velocity_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(
                      start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_EFFORT)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        velocity_controller_running_ = false;
        cartesian_motion_controller_running_ = false;
        // Leaving Cartesian control; gate the Cartesian write() path off again.
        cartesian_mode_active_ = false;

        // Hold joints when starting joint torque controller before user
        // commands arrives
        SynchronizeCommandsWithState();

        // Set to joint torque mode. This is also the step that brings the robot back from IDLE to
        // RT_JOINT_TORQUE after a fault: recovery leaves the robot operational in IDLE, and
        // restarting the effort controller lands here with a freshly synchronized command buffer.
        robot_->SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);

        // The joint impedance properties do not govern RT_JOINT_TORQUE, so what the driver holds is
        // no longer in effect while the effort controller runs.
        if (joint_impedance_config_node_) {
            joint_impedance_config_node_->MarkNotInEffect();
        }

        torque_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(start_modes_.begin(), start_modes_.end(), kStartModeCartesianPose)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        velocity_controller_running_ = false;
        torque_controller_running_ = false;

        // Hold the current TCP pose until the controller publishes its first setpoint.
        init_tcp_pose_ = robot_->states().tcp_pose;
        SynchronizeCommandsWithState();

        // Enter Cartesian mode if the driver was started in a joint mode. Without this, write()
        // drops every Cartesian command, because it requires both cartesian_mode_active_ and
        // robot_->mode() == RT_CARTESIAN_MOTION_FORCE. The joint branches above switch modes here
        // in the same way; the RDK transition blocks this update cycle, which shows up as a single
        // missed deadline on the 1 kHz loop at the moment of the switch.
        if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
            if (!runtime_cartesian_switching_) {
                RCLCPP_ERROR(getLogger(),
                    "Cartesian controller activated but the hardware was started in a joint mode "
                    "without runtime_cartesian_switching=true. Refusing to switch: the "
                    "force/torque sensor has not been zeroed for Cartesian control.");
                driver_status_->commands_synchronized.store(false);
                start_modes_.clear();
                stop_modes_.clear();
                return hardware_interface::return_type::ERROR;
            }
            robot_->SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
            robot_->SetForceControlAxis(
                std::array<bool, kCartDoF> {false, false, false, false, false, false});
            RCLCPP_INFO(getLogger(), "Switched to RT_CARTESIAN_MOTION_FORCE mode");
        }
        cartesian_mode_active_ = true;

        // The joint impedance properties do not govern RT_CARTESIAN_MOTION_FORCE, so what the
        // driver holds is no longer in effect while the Cartesian controller runs.
        if (joint_impedance_config_node_) {
            joint_impedance_config_node_->MarkNotInEffect();
        }

        RCLCPP_INFO(getLogger(),
            "Cartesian motion controller activated. Initial TCP pose: [%.4f, %.4f, %.4f, %.4f, "
            "%.4f, %.4f, %.4f]",
            init_tcp_pose_[0], init_tcp_pose_[1], init_tcp_pose_[2], init_tcp_pose_[3],
            init_tcp_pose_[4], init_tcp_pose_[5], init_tcp_pose_[6]);

        cartesian_motion_controller_running_ = true;
    }

    start_modes_.clear();
    stop_modes_.clear();

    return hardware_interface::return_type::OK;
}

bool FlexivHardwareInterface::IsCartesianCommandValid() const
{
    for (size_t i = 0; i < kCartPoseSize; i++) {
        if (std::isnan(hw_commands_cartesian_pose_[i])) {
            return false;
        }
    }
    return true;
}

void FlexivHardwareInterface::SetCartesianImpedanceCallback(
    const std::shared_ptr<flexiv_msgs::srv::SetCartesianImpedance::Request> request,
    std::shared_ptr<flexiv_msgs::srv::SetCartesianImpedance::Response> response)
{
    if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
        response->success = false;
        response->message = "Robot is not in Cartesian motion-force control mode";
        return;
    }

    try {
        std::array<double, kCartDoF> stiffness;
        std::array<double, kCartDoF> damping_ratio;
        for (size_t i = 0; i < kCartDoF; i++) {
            stiffness[i] = request->stiffness[i];
            damping_ratio[i] = request->damping_ratio[i];
        }

        robot_->SetCartesianImpedance(stiffness, damping_ratio);

        response->success = true;
        response->message = "Cartesian impedance set successfully";
        RCLCPP_INFO(getLogger(),
            "SetCartesianImpedance: stiffness=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]", stiffness[0],
            stiffness[1], stiffness[2], stiffness[3], stiffness[4], stiffness[5]);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to set Cartesian impedance: ") + e.what();
        RCLCPP_ERROR(getLogger(), "%s", response->message.c_str());
    }
}

void FlexivHardwareInterface::SetNullSpacePostureCallback(
    const std::shared_ptr<flexiv_msgs::srv::SetNullSpacePosture::Request> request,
    std::shared_ptr<flexiv_msgs::srv::SetNullSpacePosture::Response> response)
{
    if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
        response->success = false;
        response->message = "Robot is not in Cartesian motion-force control mode";
        return;
    }

    try {
        const std::vector<double> ref_positions(
            request->ref_positions.begin(), request->ref_positions.end());

        robot_->SetNullSpacePosture(ref_positions);

        response->success = true;
        response->message = "Null space posture set successfully";
        RCLCPP_INFO(getLogger(), "SetNullSpacePosture: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            ref_positions[0], ref_positions[1], ref_positions[2], ref_positions[3],
            ref_positions[4], ref_positions[5], ref_positions[6]);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to set null space posture: ") + e.what();
        RCLCPP_ERROR(getLogger(), "%s", response->message.c_str());
    }
}

void FlexivHardwareInterface::SetMaxContactWrenchCallback(
    const std::shared_ptr<flexiv_msgs::srv::SetMaxContactWrench::Request> request,
    std::shared_ptr<flexiv_msgs::srv::SetMaxContactWrench::Response> response)
{
    if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
        response->success = false;
        response->message = "Robot is not in Cartesian motion-force control mode";
        return;
    }

    try {
        std::array<double, kCartDoF> max_wrench;
        for (size_t i = 0; i < kCartDoF; i++) {
            max_wrench[i] = request->max_wrench[i];
        }

        robot_->SetMaxContactWrench(max_wrench);

        response->success = true;
        response->message = "Max contact wrench set successfully";
        RCLCPP_INFO(getLogger(), "SetMaxContactWrench: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
            max_wrench[0], max_wrench[1], max_wrench[2], max_wrench[3], max_wrench[4],
            max_wrench[5]);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to set max contact wrench: ") + e.what();
        RCLCPP_ERROR(getLogger(), "%s", response->message.c_str());
    }
}

void FlexivHardwareInterface::SetForceControlFrameCallback(
    const std::shared_ptr<flexiv_msgs::srv::SetForceControlFrame::Request> request,
    std::shared_ptr<flexiv_msgs::srv::SetForceControlFrame::Response> response)
{
    if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
        response->success = false;
        response->message = "Robot is not in Cartesian motion-force control mode";
        return;
    }

    try {
        flexiv::rdk::CoordType frame;
        if (request->frame == "world" || request->frame == "WORLD") {
            frame = flexiv::rdk::CoordType::WORLD;
        } else if (request->frame == "tcp" || request->frame == "TCP") {
            frame = flexiv::rdk::CoordType::TCP;
        } else {
            response->success = false;
            response->message = "Invalid frame. Use 'world' or 'tcp'";
            return;
        }

        robot_->SetForceControlFrame(frame);

        response->success = true;
        response->message = "Force control frame set to: " + request->frame;
        RCLCPP_INFO(getLogger(), "SetForceControlFrame: %s", request->frame.c_str());
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to set force control frame: ") + e.what();
        RCLCPP_ERROR(getLogger(), "%s", response->message.c_str());
    }
}

void FlexivHardwareInterface::SetForceControlAxisCallback(
    const std::shared_ptr<flexiv_msgs::srv::SetForceControlAxis::Request> request,
    std::shared_ptr<flexiv_msgs::srv::SetForceControlAxis::Response> response)
{
    if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
        response->success = false;
        response->message = "Robot is not in Cartesian motion-force control mode";
        return;
    }

    try {
        std::array<bool, kCartDoF> force_axes;
        for (size_t i = 0; i < kCartDoF; i++) {
            force_axes[i] = request->enable_force_control[i];
        }

        robot_->SetForceControlAxis(force_axes);

        response->success = true;
        response->message = "Force control axes set successfully";
        RCLCPP_INFO(getLogger(), "SetForceControlAxis: [%s, %s, %s, %s, %s, %s]",
            force_axes[0] ? "force" : "motion", force_axes[1] ? "force" : "motion",
            force_axes[2] ? "force" : "motion", force_axes[3] ? "force" : "motion",
            force_axes[4] ? "force" : "motion", force_axes[5] ? "force" : "motion");
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to set force control axes: ") + e.what();
        RCLCPP_ERROR(getLogger(), "%s", response->message.c_str());
    }
}

} /* namespace flexiv_hardware */

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    flexiv_hardware::FlexivHardwareInterface, hardware_interface::SystemInterface)
