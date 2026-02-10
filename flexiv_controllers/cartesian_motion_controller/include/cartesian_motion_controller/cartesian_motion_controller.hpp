/**
 * @file cartesian_motion_controller.hpp
 * @brief ROS2 controller for Flexiv robot Cartesian motion-force control.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef CARTESIAN_MOTION_CONTROLLER__CARTESIAN_MOTION_CONTROLLER_HPP_
#define CARTESIAN_MOTION_CONTROLLER__CARTESIAN_MOTION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <flexiv_msgs/msg/cartesian_motion_force_command.hpp>

#include <cartesian_motion_controller/cartesian_motion_controller_parameters.hpp>

namespace cartesian_motion_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CartesianMotionController : public controller_interface::ControllerInterface
{
public:
    CartesianMotionController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        cartesian_pose_command_interfaces_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        cartesian_wrench_command_interfaces_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        cartesian_pose_state_interfaces_;

    rclcpp::Subscription<flexiv_msgs::msg::CartesianMotionForceCommand>::SharedPtr command_sub_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<flexiv_msgs::msg::CartesianMotionForceCommand>>
        rt_command_ptr_;

    using StatePublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> state_publisher_;
    std::unique_ptr<StatePublisher> realtime_state_publisher_;

    static constexpr size_t kCartPoseSize = 7;
    static constexpr size_t kCartDoF = 6;

    const std::vector<std::string> cartesian_pose_interface_names_ = {
        "cartesian_pose_x", "cartesian_pose_y", "cartesian_pose_z",
        "cartesian_pose_qw", "cartesian_pose_qx", "cartesian_pose_qy", "cartesian_pose_qz"};

    const std::vector<std::string> cartesian_wrench_interface_names_ = {
        "cartesian_wrench_fx", "cartesian_wrench_fy", "cartesian_wrench_fz",
        "cartesian_wrench_mx", "cartesian_wrench_my", "cartesian_wrench_mz"};

    void commandCallback(
        const std::shared_ptr<flexiv_msgs::msg::CartesianMotionForceCommand> msg);
};

} // namespace cartesian_motion_controller

#endif // CARTESIAN_MOTION_CONTROLLER__CARTESIAN_MOTION_CONTROLLER_HPP_
