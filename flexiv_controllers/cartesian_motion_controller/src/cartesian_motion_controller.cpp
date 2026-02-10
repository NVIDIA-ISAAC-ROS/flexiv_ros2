/**
 * @file cartesian_motion_controller.cpp
 * @brief ROS2 controller for Flexiv robot Cartesian motion-force control
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#include "cartesian_motion_controller/cartesian_motion_controller.hpp"

#include <algorithm>
#include <cmath>

namespace cartesian_motion_controller {

CartesianMotionController::CartesianMotionController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration
CartesianMotionController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string tcp_name = params_.tcp_name;

    for (const auto& interface_name : cartesian_pose_interface_names_) {
        config.names.push_back(tcp_name + "/" + interface_name);
    }
    for (const auto& interface_name : cartesian_wrench_interface_names_) {
        config.names.push_back(tcp_name + "/" + interface_name);
    }

    return config;
}

controller_interface::InterfaceConfiguration
CartesianMotionController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string tcp_name = params_.tcp_name;

    for (const auto& interface_name : cartesian_pose_interface_names_) {
        config.names.push_back(tcp_name + "/" + interface_name);
    }

    return config;
}

CallbackReturn CartesianMotionController::on_init()
{
    try {
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Exception thrown during init: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianMotionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    params_ = param_listener_->get_params();

    if (params_.tcp_name.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'tcp_name' parameter is empty");
        return CallbackReturn::ERROR;
    }

    command_sub_ = get_node()->create_subscription<flexiv_msgs::msg::CartesianMotionForceCommand>(
        "~/command", rclcpp::SystemDefaultsQoS(),
        std::bind(&CartesianMotionController::commandCallback, this, std::placeholders::_1));

    state_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/tcp_pose", rclcpp::SystemDefaultsQoS());
    realtime_state_publisher_ = std::make_unique<StatePublisher>(state_publisher_);

    RCLCPP_INFO(get_node()->get_logger(), "Cartesian motion controller configured");
    return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianMotionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    cartesian_pose_command_interfaces_.clear();
    cartesian_wrench_command_interfaces_.clear();
    cartesian_pose_state_interfaces_.clear();

    const std::string tcp_name = params_.tcp_name;

    for (const auto& interface_name : cartesian_pose_interface_names_) {
        auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
            [&](const auto& interface) {
                return interface.get_prefix_name() == tcp_name
                       && interface.get_interface_name() == interface_name;
            });
        if (it != command_interfaces_.end()) {
            cartesian_pose_command_interfaces_.emplace_back(*it);
        } else {
            RCLCPP_ERROR(get_node()->get_logger(),
                "Could not find command interface: %s/%s", tcp_name.c_str(), interface_name.c_str());
            return CallbackReturn::ERROR;
        }
    }

    for (const auto& interface_name : cartesian_wrench_interface_names_) {
        auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
            [&](const auto& interface) {
                return interface.get_prefix_name() == tcp_name
                       && interface.get_interface_name() == interface_name;
            });
        if (it != command_interfaces_.end()) {
            cartesian_wrench_command_interfaces_.emplace_back(*it);
        } else {
            RCLCPP_ERROR(get_node()->get_logger(),
                "Could not find command interface: %s/%s", tcp_name.c_str(), interface_name.c_str());
            return CallbackReturn::ERROR;
        }
    }

    for (const auto& interface_name : cartesian_pose_interface_names_) {
        auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
            [&](const auto& interface) {
                return interface.get_prefix_name() == tcp_name
                       && interface.get_interface_name() == interface_name;
            });
        if (it != state_interfaces_.end()) {
            cartesian_pose_state_interfaces_.emplace_back(*it);
        } else {
            RCLCPP_ERROR(get_node()->get_logger(),
                "Could not find state interface: %s/%s", tcp_name.c_str(), interface_name.c_str());
            return CallbackReturn::ERROR;
        }
    }

    rt_command_ptr_.reset();

    RCLCPP_INFO(get_node()->get_logger(), "Cartesian motion controller activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianMotionController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    cartesian_pose_command_interfaces_.clear();
    cartesian_wrench_command_interfaces_.clear();
    cartesian_pose_state_interfaces_.clear();

    RCLCPP_INFO(get_node()->get_logger(), "Cartesian motion controller deactivated");
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianMotionController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    auto command_ptr = rt_command_ptr_.readFromRT();

    if (command_ptr && *command_ptr) {
        const auto& cmd = *command_ptr;

        if (cartesian_pose_command_interfaces_.size() == kCartPoseSize) {
            (void)cartesian_pose_command_interfaces_[0].get().set_value(cmd->target_pose.position.x);
            (void)cartesian_pose_command_interfaces_[1].get().set_value(cmd->target_pose.position.y);
            (void)cartesian_pose_command_interfaces_[2].get().set_value(cmd->target_pose.position.z);
            (void)cartesian_pose_command_interfaces_[3].get().set_value(cmd->target_pose.orientation.w);
            (void)cartesian_pose_command_interfaces_[4].get().set_value(cmd->target_pose.orientation.x);
            (void)cartesian_pose_command_interfaces_[5].get().set_value(cmd->target_pose.orientation.y);
            (void)cartesian_pose_command_interfaces_[6].get().set_value(cmd->target_pose.orientation.z);
        }

        if (cartesian_wrench_command_interfaces_.size() == kCartDoF) {
            (void)cartesian_wrench_command_interfaces_[0].get().set_value(cmd->target_wrench.force.x);
            (void)cartesian_wrench_command_interfaces_[1].get().set_value(cmd->target_wrench.force.y);
            (void)cartesian_wrench_command_interfaces_[2].get().set_value(cmd->target_wrench.force.z);
            (void)cartesian_wrench_command_interfaces_[3].get().set_value(cmd->target_wrench.torque.x);
            (void)cartesian_wrench_command_interfaces_[4].get().set_value(cmd->target_wrench.torque.y);
            (void)cartesian_wrench_command_interfaces_[5].get().set_value(cmd->target_wrench.torque.z);
        }
    }

    if (realtime_state_publisher_ && realtime_state_publisher_->trylock()) {
        auto& msg = realtime_state_publisher_->msg_;
        msg.header.stamp = time;
        msg.header.frame_id = "world";

        if (cartesian_pose_state_interfaces_.size() == kCartPoseSize) {
            msg.pose.position.x = cartesian_pose_state_interfaces_[0].get().get_optional<double>().value_or(0.0);
            msg.pose.position.y = cartesian_pose_state_interfaces_[1].get().get_optional<double>().value_or(0.0);
            msg.pose.position.z = cartesian_pose_state_interfaces_[2].get().get_optional<double>().value_or(0.0);
            msg.pose.orientation.w = cartesian_pose_state_interfaces_[3].get().get_optional<double>().value_or(1.0);
            msg.pose.orientation.x = cartesian_pose_state_interfaces_[4].get().get_optional<double>().value_or(0.0);
            msg.pose.orientation.y = cartesian_pose_state_interfaces_[5].get().get_optional<double>().value_or(0.0);
            msg.pose.orientation.z = cartesian_pose_state_interfaces_[6].get().get_optional<double>().value_or(0.0);
        }

        realtime_state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

void CartesianMotionController::commandCallback(
    const std::shared_ptr<flexiv_msgs::msg::CartesianMotionForceCommand> msg)
{
    rt_command_ptr_.writeFromNonRT(msg);
}

} // namespace cartesian_motion_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    cartesian_motion_controller::CartesianMotionController,
    controller_interface::ControllerInterface)
