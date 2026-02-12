/**
 * @file flexiv_hardware_interface.cpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/rrbot_system_multi_interface.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "flexiv/rdk/robot.hpp"
#include "flexiv_hardware/flexiv_hardware_interface.hpp"

namespace {

constexpr double kMaxJointVelocity = 2.0;
constexpr double kMaxJointAcceleration = 3.0;

// Custom command interface names for Cartesian control
const std::string HW_IF_CARTESIAN_POSE_X = "cartesian_pose_x";
const std::string HW_IF_CARTESIAN_POSE_Y = "cartesian_pose_y";
const std::string HW_IF_CARTESIAN_POSE_Z = "cartesian_pose_z";
const std::string HW_IF_CARTESIAN_POSE_QW = "cartesian_pose_qw";
const std::string HW_IF_CARTESIAN_POSE_QX = "cartesian_pose_qx";
const std::string HW_IF_CARTESIAN_POSE_QY = "cartesian_pose_qy";
const std::string HW_IF_CARTESIAN_POSE_QZ = "cartesian_pose_qz";
const std::string HW_IF_CARTESIAN_WRENCH_FX = "cartesian_wrench_fx";
const std::string HW_IF_CARTESIAN_WRENCH_FY = "cartesian_wrench_fy";
const std::string HW_IF_CARTESIAN_WRENCH_FZ = "cartesian_wrench_fz";
const std::string HW_IF_CARTESIAN_WRENCH_MX = "cartesian_wrench_mx";
const std::string HW_IF_CARTESIAN_WRENCH_MY = "cartesian_wrench_my";
const std::string HW_IF_CARTESIAN_WRENCH_MZ = "cartesian_wrench_mz";

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
    controllers_initialized_ = false;

    hw_commands_cartesian_pose_.fill(std::numeric_limits<double>::quiet_NaN());
    hw_commands_cartesian_wrench_.fill(0.0);
    hw_commands_cartesian_velocity_.fill(0.0);
    hw_commands_cartesian_acceleration_.fill(0.0);
    hw_states_cartesian_pose_.fill(0.0);
    hw_states_cartesian_velocity_.fill(0.0);
    init_tcp_pose_.fill(0.0);

    if (info_.joints.size() != kJointDoF) {
        RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(), kJointDoF);
        return hardware_interface::CallbackReturn::ERROR;
    }

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

    std::string robot_sn;
    try {
        robot_sn = info_.hardware_parameters["robot_sn"];
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

    try {
        RCLCPP_INFO(getLogger(), "Connecting to robot %s ...", robot_sn.c_str());
        robot_ = std::make_unique<flexiv::rdk::Robot>(robot_sn);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not connect to robot");
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
    return hardware_interface::CallbackReturn::SUCCESS;
}

rclcpp::Logger FlexivHardwareInterface::getLogger()
{
    return rclcpp::get_logger("FlexivHardwareInterface");
}

std::vector<hardware_interface::StateInterface> FlexivHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_states_joint_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_states_joint_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_joint_efforts_[i]));
    }

    std::string robot_sn = info_.hardware_parameters.at("robot_sn");
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        robot_sn, "flexiv_robot_states", reinterpret_cast<double*>(&hw_flexiv_robot_states_addr_)));

    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (std::size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            prefix + "gpio", "digital_input_" + std::to_string(i), &hw_states_gpio_in_[i]));
    }

    // Cartesian TCP pose states
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_X, &hw_states_cartesian_pose_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_Y, &hw_states_cartesian_pose_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_Z, &hw_states_cartesian_pose_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_QW, &hw_states_cartesian_pose_[3]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_QX, &hw_states_cartesian_pose_[4]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_QY, &hw_states_cartesian_pose_[5]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_QZ, &hw_states_cartesian_pose_[6]));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FlexivHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(getLogger(), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_joint_positions_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_commands_joint_velocities_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT, &hw_commands_joint_efforts_[i]));
    }

    const std::string prefix = info_.hardware_parameters.at("prefix");
    for (size_t i = 0; i < flexiv::rdk::kIOPorts; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            prefix + "gpio", "digital_output_" + std::to_string(i), &hw_commands_gpio_out_[i]));
    }

    // Cartesian TCP pose commands
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_X, &hw_commands_cartesian_pose_[0]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_Y, &hw_commands_cartesian_pose_[1]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_Z, &hw_commands_cartesian_pose_[2]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_QW, &hw_commands_cartesian_pose_[3]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_QX, &hw_commands_cartesian_pose_[4]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_QY, &hw_commands_cartesian_pose_[5]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_POSE_QZ, &hw_commands_cartesian_pose_[6]));

    // Cartesian TCP wrench commands
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_WRENCH_FX, &hw_commands_cartesian_wrench_[0]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_WRENCH_FY, &hw_commands_cartesian_wrench_[1]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_WRENCH_FZ, &hw_commands_cartesian_wrench_[2]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_WRENCH_MX, &hw_commands_cartesian_wrench_[3]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_WRENCH_MY, &hw_commands_cartesian_wrench_[4]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        prefix + "tcp", HW_IF_CARTESIAN_WRENCH_MZ, &hw_commands_cartesian_wrench_[5]));

    return command_interfaces;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Starting... please wait...");

    try {
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

        // Check the DoF of the robot
        if (robot_->info().DoF != kJointDoF) {
            RCLCPP_FATAL(getLogger(),
                "Robot has %ld DoF. Expected %ld. External axes control is not supported in ROS 2 "
                "yet.",
                robot_->info().DoF, kJointDoF);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Enable the robot
        RCLCPP_INFO(getLogger(), "Enabling robot ...");
        robot_->Enable();

        // Wait for the robot to become operational
        while (!robot_->operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(getLogger(), "Robot is now operational");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(getLogger(), "Could not enable robot.");
        RCLCPP_FATAL(getLogger(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "System successfully started!");

    // Switch to Cartesian mode early (before RT loop) to avoid blocking 1kHz control
    if (rdk_control_mode_ == flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
        // Zero force-torque sensor
        RCLCPP_WARN(getLogger(),
            "Zeroing force/torque sensor. Make sure nothing is in contact with the robot.");
        try {
            robot_->SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
            robot_->ExecutePrimitive("ZeroFTSensor", std::map<std::string, flexiv::rdk::FlexivDataTypes>{});
            
            // Wait for primitive to finish
            while (!std::get<int>(robot_->primitive_states()["terminated"])) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            RCLCPP_INFO(getLogger(), "Sensor zeroing complete");
        } catch (const std::exception& e) {
            RCLCPP_FATAL(getLogger(), "Failed to zero force/torque sensor");
            RCLCPP_FATAL(getLogger(), e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        init_tcp_pose_ = robot_->states().tcp_pose;
        robot_->SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
        robot_->SetForceControlAxis(
            std::array<bool, kCartDoF>{false, false, false, false, false, false});
        cartesian_mode_active_ = true;
        RCLCPP_INFO(getLogger(), "Switched to RT_CARTESIAN_MOTION_FORCE mode");
        RCLCPP_INFO(getLogger(), "Initial TCP pose: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
            init_tcp_pose_[0], init_tcp_pose_[1], init_tcp_pose_[2],
            init_tcp_pose_[3], init_tcp_pose_[4], init_tcp_pose_[5], init_tcp_pose_[6]);

        // Create Cartesian configuration services on a dedicated node/thread
        const std::string prefix = info_.hardware_parameters.at("prefix");
        std::string sanitized_prefix = prefix;
        std::replace(sanitized_prefix.begin(), sanitized_prefix.end(), '-', '_');
        const std::string node_name = sanitized_prefix.empty()
            ? "flexiv_hardware_services"
            : sanitized_prefix + "flexiv_hardware_services";
        service_node_ = rclcpp::Node::make_shared(node_name);

        set_cartesian_impedance_srv_ = service_node_->create_service<flexiv_msgs::srv::SetCartesianImpedance>(
            "~/set_cartesian_impedance",
            std::bind(&FlexivHardwareInterface::setCartesianImpedanceCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        set_null_space_posture_srv_ = service_node_->create_service<flexiv_msgs::srv::SetNullSpacePosture>(
            "~/set_null_space_posture",
            std::bind(&FlexivHardwareInterface::setNullSpacePostureCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        set_max_contact_wrench_srv_ = service_node_->create_service<flexiv_msgs::srv::SetMaxContactWrench>(
            "~/set_max_contact_wrench",
            std::bind(&FlexivHardwareInterface::setMaxContactWrenchCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        set_force_control_frame_srv_ = service_node_->create_service<flexiv_msgs::srv::SetForceControlFrame>(
            "~/set_force_control_frame",
            std::bind(&FlexivHardwareInterface::setForceControlFrameCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        set_force_control_axis_srv_ = service_node_->create_service<flexiv_msgs::srv::SetForceControlAxis>(
            "~/set_force_control_axis",
            std::bind(&FlexivHardwareInterface::setForceControlAxisCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        service_thread_running_ = true;
        service_thread_ = std::thread([this]() {
            while (service_thread_running_) {
                rclcpp::spin_some(service_node_);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        RCLCPP_INFO(getLogger(), "Cartesian configuration services available:");
        RCLCPP_INFO(getLogger(), "  - %s/set_cartesian_impedance", node_name.c_str());
        RCLCPP_INFO(getLogger(), "  - %s/set_null_space_posture", node_name.c_str());
        RCLCPP_INFO(getLogger(), "  - %s/set_max_contact_wrench", node_name.c_str());
        RCLCPP_INFO(getLogger(), "  - %s/set_force_control_frame", node_name.c_str());
        RCLCPP_INFO(getLogger(), "  - %s/set_force_control_axis", node_name.c_str());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FlexivHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Stopping... please wait...");

    service_thread_running_ = false;
    if (service_thread_.joinable()) {
        service_thread_.join();
    }
    set_cartesian_impedance_srv_.reset();
    set_null_space_posture_srv_.reset();
    set_max_contact_wrench_srv_.reset();
    set_force_control_frame_srv_.reset();
    set_force_control_axis_srv_.reset();
    service_node_.reset();

    cartesian_mode_active_ = false;

    robot_->Stop();

    RCLCPP_INFO(getLogger(), "System successfully stopped!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FlexivHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (robot_->operational()) {

        hw_flexiv_robot_states_ = robot_->states();

        for (size_t i = 0; i < info_.joints.size(); i++) {
            hw_states_joint_positions_[i] = robot_->states().q[i];
            hw_states_joint_velocities_[i] = robot_->states().dtheta[i];
            hw_states_joint_efforts_[i] = robot_->states().tau[i];
        }

        const auto& tcp_pose = robot_->states().tcp_pose;
        for (size_t i = 0; i < kCartPoseSize; i++) {
            hw_states_cartesian_pose_[i] = tcp_pose[i];
        }

        const auto& tcp_vel = robot_->states().tcp_vel;
        for (size_t i = 0; i < kCartDoF; i++) {
            hw_states_cartesian_velocity_[i] = tcp_vel[i];
        }

        auto gpio_in = robot_->digital_inputs();
        for (size_t i = 0; i < hw_states_gpio_in_.size(); i++) {
            hw_states_gpio_in_[i] = static_cast<double>(gpio_in[i]);
        }
    }

    return hardware_interface::return_type::OK;
}

bool FlexivHardwareInterface::isCartesianCommandValid() const
{
    for (size_t i = 0; i < kCartPoseSize; i++) {
        if (std::isnan(hw_commands_cartesian_pose_[i])) {
            return false;
        }
    }
    return true;
}

hardware_interface::return_type FlexivHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::vector<double> target_pos(robot_->info().DoF);
    std::vector<double> target_vel(robot_->info().DoF);

    std::vector<double> max_vel(robot_->info().DoF, kMaxJointVelocity);
    std::vector<double> max_acc(robot_->info().DoF, kMaxJointAcceleration);

    bool isNanPos = false;
    bool isNanVel = false;
    bool isNanEff = false;
    for (std::size_t i = 0; i < robot_->info().DoF; i++) {
        if (hw_commands_joint_positions_[i] != hw_commands_joint_positions_[i]) {
            isNanPos = true;
        }
        if (hw_commands_joint_velocities_[i] != hw_commands_joint_velocities_[i]) {
            isNanVel = true;
        }
        if (hw_commands_joint_efforts_[i] != hw_commands_joint_efforts_[i]) {
            isNanEff = true;
        }
    }

    if (position_controller_running_ && robot_->mode() == rdk_control_mode_ && !isNanPos) {
        target_pos = hw_commands_joint_positions_;
        robot_->SendJointPosition(target_pos, target_vel, max_vel, max_acc);
    } else if (velocity_controller_running_ && robot_->mode() == rdk_control_mode_ && !isNanVel) {
        target_pos = hw_states_joint_positions_;
        target_vel = hw_commands_joint_velocities_;
        robot_->SendJointPosition(target_pos, target_vel, max_vel, max_acc);
    } else if (torque_controller_running_ && robot_->mode() == flexiv::rdk::Mode::RT_JOINT_TORQUE
               && !isNanEff) {
        std::vector<double> target_torque(robot_->info().DoF);
        target_torque = hw_commands_joint_efforts_;
        robot_->StreamJointTorque(target_torque, true, true);
    } else if (cartesian_mode_active_
               && robot_->mode() == flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
        if (cartesian_motion_controller_running_ && isCartesianCommandValid()) {
            robot_->StreamCartesianMotionForce(
                hw_commands_cartesian_pose_, hw_commands_cartesian_wrench_,
                hw_commands_cartesian_velocity_, hw_commands_cartesian_acceleration_);
        } else {
            std::array<double, kCartDoF> zero_wrench = {};
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

hardware_interface::return_type FlexivHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    start_modes_.clear();
    stop_modes_.clear();

    const std::string prefix = info_.hardware_parameters.at("prefix");

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
        if (key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_X
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_Y
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_Z
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_QW
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_QX
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_QY
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_QZ) {
            start_modes_.push_back("cartesian_pose");
        }
        if (key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_FX
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_FY
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_FZ
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_MX
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_MY
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_MZ) {
            start_modes_.push_back("cartesian_wrench");
        }
    }

    size_t cartesian_pose_count = std::count(start_modes_.begin(), start_modes_.end(), "cartesian_pose");
    size_t cartesian_wrench_count = std::count(start_modes_.begin(), start_modes_.end(), "cartesian_wrench");
    size_t cartesian_count = cartesian_pose_count + cartesian_wrench_count;
    if (cartesian_pose_count > 0 && cartesian_pose_count != kCartPoseSize) {
        RCLCPP_ERROR(getLogger(), "All Cartesian pose interfaces must be claimed together");
        return hardware_interface::return_type::ERROR;
    }
    if (cartesian_wrench_count > 0 && cartesian_wrench_count != kCartDoF) {
        RCLCPP_ERROR(getLogger(), "All Cartesian wrench interfaces must be claimed together");
        return hardware_interface::return_type::ERROR;
    }

    size_t joint_start_count = start_modes_.size() - cartesian_count;
    if (joint_start_count != 0 && joint_start_count != info_.joints.size()) {
        return hardware_interface::return_type::ERROR;
    }
    if (joint_start_count != 0 && cartesian_count == 0
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
        if (key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_X
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_Y
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_Z
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_QW
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_QX
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_QY
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_POSE_QZ
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_FX
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_FY
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_FZ
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_MX
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_MY
            || key == prefix + "tcp/" + HW_IF_CARTESIAN_WRENCH_MZ) {
            stop_modes_.push_back(StoppingInterface::STOP_CARTESIAN);
        }
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
        robot_->Stop();
    } else if (stop_modes_.size() != 0
               && std::find(
                      stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY)
                      != stop_modes_.end()) {
        velocity_controller_running_ = false;
        robot_->Stop();
    } else if (stop_modes_.size() != 0
               && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_EFFORT)
                      != stop_modes_.end()) {
        torque_controller_running_ = false;
        robot_->Stop();
    } else if (stop_modes_.size() != 0
               && std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_CARTESIAN)
                      != stop_modes_.end()) {
        cartesian_motion_controller_running_ = false;
        robot_->Stop();
    }

    if (start_modes_.size() != 0
        && std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION)
               != start_modes_.end()) {
        velocity_controller_running_ = false;
        torque_controller_running_ = false;
        cartesian_motion_controller_running_ = false;

        // Hold joints before user commands arrives
        std::fill(hw_commands_joint_positions_.begin(), hw_commands_joint_positions_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint position or joint impedance mode
        robot_->SwitchMode(rdk_control_mode_);

        position_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(
                      start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_VELOCITY)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        torque_controller_running_ = false;
        cartesian_motion_controller_running_ = false;

        // Hold joints before user commands arrives
        std::fill(hw_commands_joint_velocities_.begin(), hw_commands_joint_velocities_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint position or joint impedance mode
        robot_->SwitchMode(rdk_control_mode_);

        velocity_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(
                      start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_EFFORT)
                      != start_modes_.end()) {
        position_controller_running_ = false;
        velocity_controller_running_ = false;
        cartesian_motion_controller_running_ = false;

        // Hold joints when starting joint torque controller before user
        // commands arrives
        std::fill(hw_commands_joint_efforts_.begin(), hw_commands_joint_efforts_.end(),
            std::numeric_limits<double>::quiet_NaN());

        // Set to joint torque mode
        robot_->SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);

        torque_controller_running_ = true;
    } else if (start_modes_.size() != 0
               && std::find(start_modes_.begin(), start_modes_.end(), "cartesian_pose")
                      != start_modes_.end()) {
        position_controller_running_ = false;
        velocity_controller_running_ = false;
        torque_controller_running_ = false;

        init_tcp_pose_ = robot_->states().tcp_pose;
        hw_commands_cartesian_pose_.fill(std::numeric_limits<double>::quiet_NaN());
        hw_commands_cartesian_wrench_.fill(0.0);
        hw_commands_cartesian_velocity_.fill(0.0);
        hw_commands_cartesian_acceleration_.fill(0.0);

        RCLCPP_INFO(getLogger(), "Cartesian motion controller activated");
        RCLCPP_INFO(getLogger(), "Initial TCP pose: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
            init_tcp_pose_[0], init_tcp_pose_[1], init_tcp_pose_[2],
            init_tcp_pose_[3], init_tcp_pose_[4], init_tcp_pose_[5], init_tcp_pose_[6]);

        cartesian_motion_controller_running_ = true;
    }

    start_modes_.clear();
    stop_modes_.clear();

    return hardware_interface::return_type::OK;
}

void FlexivHardwareInterface::setCartesianImpedanceCallback(
    const std::shared_ptr<flexiv_msgs::srv::SetCartesianImpedance::Request> request,
    std::shared_ptr<flexiv_msgs::srv::SetCartesianImpedance::Response> response)
{
    try {
        if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
            response->success = false;
            response->message = "Robot is not in Cartesian motion-force control mode";
            return;
        }

        std::array<double, kCartDoF> stiffness;
        std::array<double, kCartDoF> damping_ratio;
        for (size_t i = 0; i < kCartDoF; i++) {
            stiffness[i] = request->stiffness[i];
            damping_ratio[i] = request->damping_ratio[i];
        }

        robot_->SetCartesianImpedance(stiffness, damping_ratio);

        response->success = true;
        response->message = "Cartesian impedance set successfully";
        RCLCPP_INFO(getLogger(), "SetCartesianImpedance: stiffness=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
            stiffness[0], stiffness[1], stiffness[2], stiffness[3], stiffness[4], stiffness[5]);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to set Cartesian impedance: ") + e.what();
        RCLCPP_ERROR(getLogger(), "%s", response->message.c_str());
    }
}

void FlexivHardwareInterface::setNullSpacePostureCallback(
    const std::shared_ptr<flexiv_msgs::srv::SetNullSpacePosture::Request> request,
    std::shared_ptr<flexiv_msgs::srv::SetNullSpacePosture::Response> response)
{
    try {
        if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
            response->success = false;
            response->message = "Robot is not in Cartesian motion-force control mode";
            return;
        }

        std::vector<double> ref_positions(request->ref_positions.begin(),
                                           request->ref_positions.end());

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

void FlexivHardwareInterface::setMaxContactWrenchCallback(
    const std::shared_ptr<flexiv_msgs::srv::SetMaxContactWrench::Request> request,
    std::shared_ptr<flexiv_msgs::srv::SetMaxContactWrench::Response> response)
{
    try {
        if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
            response->success = false;
            response->message = "Robot is not in Cartesian motion-force control mode";
            return;
        }

        std::array<double, kCartDoF> max_wrench;
        for (size_t i = 0; i < kCartDoF; i++) {
            max_wrench[i] = request->max_wrench[i];
        }

        robot_->SetMaxContactWrench(max_wrench);

        response->success = true;
        response->message = "Max contact wrench set successfully";
        RCLCPP_INFO(getLogger(), "SetMaxContactWrench: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
            max_wrench[0], max_wrench[1], max_wrench[2], max_wrench[3], max_wrench[4], max_wrench[5]);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to set max contact wrench: ") + e.what();
        RCLCPP_ERROR(getLogger(), "%s", response->message.c_str());
    }
}

void FlexivHardwareInterface::setForceControlFrameCallback(
    const std::shared_ptr<flexiv_msgs::srv::SetForceControlFrame::Request> request,
    std::shared_ptr<flexiv_msgs::srv::SetForceControlFrame::Response> response)
{
    try {
        if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
            response->success = false;
            response->message = "Robot is not in Cartesian motion-force control mode";
            return;
        }

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

void FlexivHardwareInterface::setForceControlAxisCallback(
    const std::shared_ptr<flexiv_msgs::srv::SetForceControlAxis::Request> request,
    std::shared_ptr<flexiv_msgs::srv::SetForceControlAxis::Response> response)
{
    try {
        if (robot_->mode() != flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE) {
            response->success = false;
            response->message = "Robot is not in Cartesian motion-force control mode";
            return;
        }

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
