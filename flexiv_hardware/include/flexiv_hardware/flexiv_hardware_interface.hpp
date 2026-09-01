/**
 * @file flexiv_hardware_interface.hpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/include/ros2_control_demo_example_3/rrbot_system_multi_interface.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_
#define FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

// ros2_control hardware_interface
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

// Flexiv msgs
#include "flexiv_msgs/srv/set_cartesian_impedance.hpp"
#include "flexiv_msgs/srv/set_force_control_axis.hpp"
#include "flexiv_msgs/srv/set_force_control_frame.hpp"
#include "flexiv_msgs/srv/set_max_contact_wrench.hpp"
#include "flexiv_msgs/srv/set_null_space_posture.hpp"

// Flexiv
#include "flexiv/rdk/robot.hpp"

#include "flexiv_hardware/fault_recovery.hpp"
#include "flexiv_hardware/joint_impedance_config_node.hpp"
#include "flexiv_hardware/recovery_node.hpp"
#include "flexiv_hardware/robot_system_control.hpp"

namespace flexiv_hardware {

enum StoppingInterface
{
    NONE,
    STOP_POSITION,
    STOP_VELOCITY,
    STOP_EFFORT,
    STOP_CARTESIAN
};

/** Cartesian pose array size: [x, y, z, qw, qx, qy, qz] */
constexpr size_t kCartPoseSize = 7;

/** Cartesian space degrees of freedom: [Fx, Fy, Fz, Mx, My, Mz] */
constexpr size_t kCartDoF = 6;

// on_activate() runs on the controller_manager's executor thread, so anything slow here makes
// /controller_manager services unavailable for that long. Keep the force/torque zeroing retry
// budget small: the launch files order other RDK clients after activation, so a retry covers a
// brief race, not a ten-second gripper init.
constexpr int kZeroFTSensorMaxAttempts = 3;
constexpr int kZeroFTSensorRetryDelayMs = 500;
// Upper bound on the ZeroFTSensor primitive, so a primitive that never reports termination cannot
// wedge activation.
constexpr int kZeroFTSensorPrimitiveTimeoutMs = 10000;

class FlexivHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FlexivHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::InterfaceDescription>
    export_unlisted_state_interface_descriptions() override;

    std::vector<hardware_interface::InterfaceDescription>
    export_unlisted_command_interface_descriptions() override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    /**
     * @brief [Blocking] Wait for the robot to become operational, up to [timeout]. Logs the
     * operational status while waiting so a stuck robot explains itself.
     * @return True if the robot became operational, false on timeout.
     */
    bool WaitUntilOperational(std::chrono::seconds timeout);

    /**
     * @brief Set the joint command buffers to hold the currently measured position, so that
     * resuming control cannot apply a stale command.
     */
    void SynchronizeCommandsWithState();

    /**
     * @brief Notice a robot that was moved while the driver was not ready, and warn about it once
     * on the return to READY. Called from read().
     */
    void TrackPositionChangeAcrossInterruption();

    /**
     * @brief [Blocking] Stop the robot, but only if it is operational. Stop() switches the control
     * mode internally, which the robot rejects unless it is operational -- and a robot that is not
     * operational is not executing anything, so there is nothing to stop.
     */
    void StopIfOperational();

    /** @brief Tear down the recovery node and release the robot connection. */
    void Disconnect();

    /**
     * @brief Resolve the framework-owned interface handles once, in declaration order, so that
     * read() and write() need no name lookup. Called from on_configure(), by which point the
     * resource manager has already created the interfaces from the URDF.
     * @return True if every expected interface was found.
     */
    bool ResolveInterfaceHandles();

    /** @brief Copy the internal state buffers into the framework's state interfaces. */
    void PublishStatesToInterfaces();

    /** @brief Copy the framework's command interfaces into the internal command buffers. */
    void ReadCommandsFromInterfaces();

    /** @brief Copy the internal command buffers into the framework's command interfaces. */
    void PushCommandsToInterfaces();

    /**
     * @brief [Blocking] Zero the force/torque sensor, retrying a few times so that another RDK
     * client briefly holding the robot does not fail the preparation outright.
     * @param[out] error Description of the last failure, when this returns false.
     * @return True if the sensor was zeroed.
     */
    bool ZeroFTSensor(std::string& error);

    /**
     * @brief [Blocking] Prepare Cartesian motion-force control during activation: zero the
     * force/torque sensor and either enter RT_CARTESIAN_MOTION_FORCE (when the driver is Cartesian
     * for its whole lifetime) or hand the robot back to IDLE for a later runtime switch.
     * @return False only when the driver cannot come up at all.
     */
    bool PrepareCartesianControl();

    /** @brief Bring up the Cartesian configuration services on the controller manager's executor. */
    void StartCartesianConfigServices();

    /** @brief Tear down the Cartesian configuration services. */
    void StopCartesianConfigServices();

    /** @brief True when every Cartesian pose command holds a finite value. */
    bool IsCartesianCommandValid() const;

    // Flexiv RDK
    std::unique_ptr<flexiv::rdk::Robot> robot_;

    // Recovery interface, hosted on the controller manager's executor
    std::unique_ptr<RobotSystemControl> robot_system_control_;
    std::shared_ptr<DriverStatus> driver_status_;
    std::shared_ptr<RecoveryNode> recovery_node_;
    rclcpp::Executor::WeakPtr executor_;

    // Joint impedance interface, hosted on the same executor. Only brought up when the driver runs
    // in a joint impedance control mode.
    std::shared_ptr<JointImpedanceConfigNode> joint_impedance_config_node_;

    // RDK control mode for joint position and velocity interfaces
    flexiv::rdk::Mode rdk_control_mode_;

    // Joint commands
    std::vector<double> hw_commands_joint_positions_;
    std::vector<double> hw_commands_joint_velocities_;
    std::vector<double> hw_commands_joint_efforts_;

    // Joint states
    std::vector<double> hw_states_joint_positions_;
    std::vector<double> hw_states_joint_velocities_;
    std::vector<double> hw_states_joint_efforts_;

    // Cartesian commands and states, in RDK order: pose is [x, y, z, qw, qx, qy, qz] and the
    // wrench/velocity/acceleration arrays are [x, y, z, Rx, Ry, Rz].
    std::array<double, kCartPoseSize> hw_commands_cartesian_pose_;
    std::array<double, kCartDoF> hw_commands_cartesian_wrench_;
    std::array<double, kCartDoF> hw_commands_cartesian_velocity_;
    std::array<double, kCartDoF> hw_commands_cartesian_acceleration_;
    std::array<double, kCartPoseSize> hw_states_cartesian_pose_;

    // Robot States
    flexiv::rdk::RobotStates hw_flexiv_robot_states_;

    // GPIO commands and states
    std::vector<double> hw_commands_gpio_out_;

    // Joint positions as last measured before the driver left READY, for detecting a robot that
    // was moved while it was not being commanded. Empty while the driver is ready.
    std::vector<double> positions_before_interruption_;
    bool was_ready_ = false;

    std::vector<double> hw_states_gpio_in_;

    // Framework-owned interface handles, resolved once in on_configure() and stored in the same
    // order as info_.joints / info_.gpios, so that read() and write() do no name lookup and take no
    // lock on the real-time path.
    std::vector<hardware_interface::StateInterface::SharedPtr> handles_state_joint_positions_;
    std::vector<hardware_interface::StateInterface::SharedPtr> handles_state_joint_velocities_;
    std::vector<hardware_interface::StateInterface::SharedPtr> handles_state_joint_efforts_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> handles_command_joint_positions_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> handles_command_joint_velocities_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> handles_command_joint_efforts_;
    std::vector<hardware_interface::StateInterface::SharedPtr> handles_state_gpio_in_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> handles_command_gpio_out_;
    hardware_interface::StateInterface::SharedPtr handle_state_flexiv_robot_states_;
    std::vector<hardware_interface::StateInterface::SharedPtr> handles_state_cartesian_pose_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> handles_command_cartesian_pose_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> handles_command_cartesian_wrench_;

    // Map from RDK joint index to ROS joint index
    // RDK expects: [ext_axis_1, ..., ext_axis_N, arm_joint_1, ..., arm_joint_7]
    std::vector<size_t> rdk_to_ros_map_;

    // Current digital output map
    std::map<unsigned int, bool> current_digital_outputs_;

    static rclcpp::Logger getLogger();

    // Control modes
    bool controllers_initialized_;
    std::vector<uint> stop_modes_;
    std::vector<std::string> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool torque_controller_running_;
    bool cartesian_motion_controller_running_;
    // True once the robot is actually in RT_CARTESIAN_MOTION_FORCE, which write() requires before
    // streaming any Cartesian command.
    bool cartesian_mode_active_;
    // When true, the driver starts in rdk_control_mode_ (a joint mode) but is prepared to enter
    // RT_CARTESIAN_MOTION_FORCE later, when a controller claiming the tcp/cartesian_pose_*
    // interfaces is activated. Set from the "runtime_cartesian_switching" hardware parameter.
    bool runtime_cartesian_switching_;
    std::array<double, kCartPoseSize> init_tcp_pose_;

    // Cartesian configuration services, hosted on the controller manager's executor alongside the
    // recovery and joint impedance nodes.
    rclcpp::Node::SharedPtr cartesian_config_node_;
    rclcpp::Service<flexiv_msgs::srv::SetCartesianImpedance>::SharedPtr set_cartesian_impedance_srv_;
    rclcpp::Service<flexiv_msgs::srv::SetNullSpacePosture>::SharedPtr set_null_space_posture_srv_;
    rclcpp::Service<flexiv_msgs::srv::SetMaxContactWrench>::SharedPtr set_max_contact_wrench_srv_;
    rclcpp::Service<flexiv_msgs::srv::SetForceControlFrame>::SharedPtr set_force_control_frame_srv_;
    rclcpp::Service<flexiv_msgs::srv::SetForceControlAxis>::SharedPtr set_force_control_axis_srv_;

    void SetCartesianImpedanceCallback(
        const std::shared_ptr<flexiv_msgs::srv::SetCartesianImpedance::Request> request,
        std::shared_ptr<flexiv_msgs::srv::SetCartesianImpedance::Response> response);
    void SetNullSpacePostureCallback(
        const std::shared_ptr<flexiv_msgs::srv::SetNullSpacePosture::Request> request,
        std::shared_ptr<flexiv_msgs::srv::SetNullSpacePosture::Response> response);
    void SetMaxContactWrenchCallback(
        const std::shared_ptr<flexiv_msgs::srv::SetMaxContactWrench::Request> request,
        std::shared_ptr<flexiv_msgs::srv::SetMaxContactWrench::Response> response);
    void SetForceControlFrameCallback(
        const std::shared_ptr<flexiv_msgs::srv::SetForceControlFrame::Request> request,
        std::shared_ptr<flexiv_msgs::srv::SetForceControlFrame::Response> response);
    void SetForceControlAxisCallback(
        const std::shared_ptr<flexiv_msgs::srv::SetForceControlAxis::Request> request,
        std::shared_ptr<flexiv_msgs::srv::SetForceControlAxis::Response> response);
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_ */
