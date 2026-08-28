/**
 * @file flexiv_dual_hardware_interface.hpp
 * @brief Hardware interface to a pair of Flexiv robots for ROS 2 control.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__FLEXIV_DUAL_HARDWARE_INTERFACE_HPP_
#define FLEXIV_HARDWARE__FLEXIV_DUAL_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

// ROS
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

// Flexiv
#include "flexiv/drdk/robot_pair.hpp"

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
    STOP_EFFORT
};

class FlexivDualHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FlexivDualHardwareInterface)

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
     * @brief [Blocking] Wait for both robots to become operational, up to [timeout].
     * @return True if both became operational, false on timeout.
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
     * @brief [Blocking] Stop the robots, but only if the pair is operational. Stop() switches the
     * control mode internally, which the robots reject unless they are operational -- and a pair
     * that is not operational is not executing anything, so there is nothing to stop.
     */
    void StopIfOperational();

    /**
     * @brief Remove the recovery node from the executor and destroy it.
     */
    void TeardownRecoveryNode();

    // Flexiv DRDK
    std::unique_ptr<flexiv::drdk::RobotPair> robot_pair_;

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

    // External axis type
    std::string external_axis_type_ = "";

    // Joint commands
    std::vector<double> hw_commands_joint_positions_;
    std::vector<double> hw_commands_joint_velocities_;
    std::vector<double> hw_commands_joint_efforts_;

    // Joint states
    std::vector<double> hw_states_joint_positions_;
    std::vector<double> hw_states_joint_velocities_;
    std::vector<double> hw_states_joint_efforts_;

    // Robot States
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

    // Framework-owned interface handles, resolved once in on_configure() and stored in the same
    // order as info_.joints and the GPIO buffers, so that read() and write() do no name lookup and
    // take no lock on the real-time path.
    std::vector<hardware_interface::StateInterface::SharedPtr> handles_state_joint_positions_;
    std::vector<hardware_interface::StateInterface::SharedPtr> handles_state_joint_velocities_;
    std::vector<hardware_interface::StateInterface::SharedPtr> handles_state_joint_efforts_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> handles_command_joint_positions_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> handles_command_joint_velocities_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> handles_command_joint_efforts_;
    std::vector<hardware_interface::StateInterface::SharedPtr> handles_state_gpio_in_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> handles_command_gpio_out_;

    flexiv::rdk::RobotStates hw_flexiv_robot_states_left_;
    flexiv::rdk::RobotStates hw_flexiv_robot_states_right_;

    // GPIO commands and states
    std::vector<double> hw_commands_gpio_out_;

    // Joint positions as last measured before the driver left READY, for detecting a robot that
    // was moved while it was not being commanded. Empty while the driver is ready.
    std::vector<double> positions_before_interruption_;
    bool was_ready_ = false;

    std::vector<double> hw_states_gpio_in_;

    // Current digital output map
    std::map<unsigned int, bool> current_digital_outputs_left_;
    std::map<unsigned int, bool> current_digital_outputs_right_;

    // Joint mapping
    struct JointMap
    {
        int robot_index; // 0: Left, 1: Right
        int dof_index;   // Index in the robot's q vector
    };
    std::vector<JointMap> joint_map_;

    static rclcpp::Logger getLogger();

    // Control modes
    bool controllers_initialized_;
    std::vector<uint> stop_modes_;
    std::vector<std::string> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool torque_controller_running_;
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__FLEXIV_DUAL_HARDWARE_INTERFACE_HPP_ */
