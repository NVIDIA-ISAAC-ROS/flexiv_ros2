/**
 * @file flexiv_hardware_interface.hpp
 * @brief Hardware interface to Flexiv robots for ROS 2 control. Adapted from
 * ros2_control_demos/example_3/hardware/include/ros2_control_demo_example_3/rrbot_system_multi_interface.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#ifndef FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_
#define FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

// Flexiv msgs
#include "flexiv_msgs/srv/set_cartesian_impedance.hpp"
#include "flexiv_msgs/srv/set_null_space_posture.hpp"
#include "flexiv_msgs/srv/set_max_contact_wrench.hpp"
#include "flexiv_msgs/srv/set_force_control_frame.hpp"
#include "flexiv_msgs/srv/set_force_control_axis.hpp"

// ros2_control hardware_interface
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

// Flexiv
#include "flexiv/rdk/robot.hpp"

namespace flexiv_hardware {

/** Robot joint space degree of freedoms */
constexpr size_t kJointDoF = 7;

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

class FlexivHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FlexivHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

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
    // Flexiv RDK
    std::unique_ptr<flexiv::rdk::Robot> robot_;

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

    // Cartesian commands and states
    std::array<double, kCartPoseSize> hw_commands_cartesian_pose_;
    std::array<double, kCartDoF> hw_commands_cartesian_wrench_;
    std::array<double, kCartDoF> hw_commands_cartesian_velocity_;
    std::array<double, kCartDoF> hw_commands_cartesian_acceleration_;
    std::array<double, kCartPoseSize> hw_states_cartesian_pose_;
    std::array<double, kCartDoF> hw_states_cartesian_velocity_;

    // Robot States
    flexiv::rdk::RobotStates hw_flexiv_robot_states_;
    flexiv::rdk::RobotStates* hw_flexiv_robot_states_addr_ = &hw_flexiv_robot_states_;

    // Desired/commanded values (tau_d, etc.) via Robot::actions() since RDK v1.9.
    flexiv::rdk::RobotActions hw_flexiv_robot_actions_;
    flexiv::rdk::RobotActions* hw_flexiv_robot_actions_addr_ = &hw_flexiv_robot_actions_;

    // GPIO commands and states
    std::vector<double> hw_commands_gpio_out_;
    std::vector<double> hw_states_gpio_in_;

    // Current digital output map
    std::map<unsigned int, bool> current_digital_outputs_;

    static rclcpp::Logger getLogger();

    // Controller mode tracking
    bool controllers_initialized_;
    std::vector<uint> stop_modes_;
    std::vector<std::string> start_modes_;
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool torque_controller_running_;
    bool cartesian_motion_controller_running_;
    bool cartesian_mode_active_;
    // When true, the driver starts in rdk_control_mode_ (a joint mode) but is
    // prepared to enter RT_CARTESIAN_MOTION_FORCE later, when a controller
    // claiming the tcp/cartesian_pose_* interfaces is activated. Set from the
    // "runtime_cartesian_switching" hardware parameter.
    bool runtime_cartesian_switching_;
    std::array<double, kCartPoseSize> init_tcp_pose_;

    bool isCartesianCommandValid() const;

    // Cartesian configuration services (run in a dedicated thread)
    rclcpp::Node::SharedPtr service_node_;
    std::thread service_thread_;
    std::atomic<bool> service_thread_running_{false};
    rclcpp::Service<flexiv_msgs::srv::SetCartesianImpedance>::SharedPtr set_cartesian_impedance_srv_;
    rclcpp::Service<flexiv_msgs::srv::SetNullSpacePosture>::SharedPtr set_null_space_posture_srv_;
    rclcpp::Service<flexiv_msgs::srv::SetMaxContactWrench>::SharedPtr set_max_contact_wrench_srv_;
    rclcpp::Service<flexiv_msgs::srv::SetForceControlFrame>::SharedPtr set_force_control_frame_srv_;
    rclcpp::Service<flexiv_msgs::srv::SetForceControlAxis>::SharedPtr set_force_control_axis_srv_;

    void setCartesianImpedanceCallback(
        const std::shared_ptr<flexiv_msgs::srv::SetCartesianImpedance::Request> request,
        std::shared_ptr<flexiv_msgs::srv::SetCartesianImpedance::Response> response);
    void setNullSpacePostureCallback(
        const std::shared_ptr<flexiv_msgs::srv::SetNullSpacePosture::Request> request,
        std::shared_ptr<flexiv_msgs::srv::SetNullSpacePosture::Response> response);
    void setMaxContactWrenchCallback(
        const std::shared_ptr<flexiv_msgs::srv::SetMaxContactWrench::Request> request,
        std::shared_ptr<flexiv_msgs::srv::SetMaxContactWrench::Response> response);
    void setForceControlFrameCallback(
        const std::shared_ptr<flexiv_msgs::srv::SetForceControlFrame::Request> request,
        std::shared_ptr<flexiv_msgs::srv::SetForceControlFrame::Response> response);
    void setForceControlAxisCallback(
        const std::shared_ptr<flexiv_msgs::srv::SetForceControlAxis::Request> request,
        std::shared_ptr<flexiv_msgs::srv::SetForceControlAxis::Response> response);
};

} /* namespace flexiv_hardware */

#endif /* FLEXIV_HARDWARE__FLEXIV_HARDWARE_INTERFACE_HPP_ */
