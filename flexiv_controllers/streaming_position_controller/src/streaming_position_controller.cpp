/**
 * @file streaming_position_controller.cpp
 * @brief Minimal ros2_control controller: subscribes to JointState on a topic
 *        and writes target positions directly to the hardware command interface.
 *
 * Parameters:
 *   joints  (string[])  -- joint names to command
 *
 * Subscribes to:
 *   ~/joint_commands  (sensor_msgs/JointState)
 *
 */

#include "streaming_position_controller/streaming_position_controller.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace streaming_position_controller {

StreamingPositionController::StreamingPositionController()
: controller_interface::ControllerInterface(),
  positions_initialized_(false),
  command_subscriber_(nullptr)
{
}

controller_interface::CallbackReturn StreamingPositionController::on_init()
{
    try
    {
        auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StreamingPositionController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    command_interface_types_.clear();
    name_to_index_.clear();
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
        command_interface_types_.push_back(
            joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
        name_to_index_[joint_names_[i]] = i;
    }

    command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
        "~/joint_commands", rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            if (msg->name.size() != msg->position.size())
            {
                RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()),
                    1000, "JointState name/position size mismatch -- dropping");
                return;
            }
            for (const auto & p : msg->position)
            {
                if (!std::isfinite(p))
                {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()),
                        1000, "Non-finite position received -- dropping");
                    return;
                }
            }
            rt_command_.set(*msg);
        });

    RCLCPP_INFO(get_node()->get_logger(),
        "Configured with %zu joints (direct passthrough, no smoothing)",
        joint_names_.size());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
StreamingPositionController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names = command_interface_types_;
    return cfg;
}

controller_interface::InterfaceConfiguration
StreamingPositionController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_)
    {
        cfg.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return cfg;
}

controller_interface::CallbackReturn StreamingPositionController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered;
    if (!controller_interface::get_ordered_interfaces(
            command_interfaces_, command_interface_types_, std::string(""), ordered) ||
        ordered.size() != joint_names_.size())
    {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Expected %zu command interfaces, got %zu",
            joint_names_.size(), ordered.size());
        return controller_interface::CallbackReturn::ERROR;
    }

    const std::size_t n = joint_names_.size();

    pos_state_idx_.assign(n, SIZE_MAX);
    for (std::size_t si = 0; si < state_interfaces_.size(); ++si)
    {
        const auto & iface = state_interfaces_[si];
        auto jt = name_to_index_.find(iface.get_prefix_name());
        if (jt == name_to_index_.end()) { continue; }
        if (iface.get_interface_name() == hardware_interface::HW_IF_POSITION)
        {
            pos_state_idx_[jt->second] = si;
        }
    }

    for (std::size_t i = 0; i < n; ++i)
    {
        if (pos_state_idx_[i] == SIZE_MAX)
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                "Position state interface not found for joint '%s'",
                joint_names_[i].c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    current_positions_.resize(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        auto val_opt = state_interfaces_[pos_state_idx_[i]].get_optional();
        double val = val_opt.has_value() ? val_opt.value() : 0.0;
        current_positions_[i] = std::isfinite(val) ? val : 0.0;
    }
    positions_initialized_ = true;

    for (std::size_t i = 0; i < n; ++i)
    {
        (void)command_interfaces_[i].set_value(current_positions_[i]);
    }

    sensor_msgs::msg::JointState empty;
    rt_command_.try_set(empty);

    RCLCPP_INFO(get_node()->get_logger(), "Activated -- holding current position");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StreamingPositionController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    positions_initialized_ = false;
    sensor_msgs::msg::JointState empty;
    rt_command_.try_set(empty);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type StreamingPositionController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!positions_initialized_)
    {
        return controller_interface::return_type::OK;
    }

    auto cmd_opt = rt_command_.try_get();
    if (cmd_opt.has_value())
    {
        last_command_ = cmd_opt.value();
    }

    if (last_command_.name.empty())
    {
        for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            (void)command_interfaces_[i].set_value(current_positions_[i]);
        }
        return controller_interface::return_type::OK;
    }

    for (std::size_t j = 0; j < last_command_.name.size(); ++j)
    {
        auto it = name_to_index_.find(last_command_.name[j]);
        if (it != name_to_index_.end() && j < last_command_.position.size())
        {
            current_positions_[it->second] = last_command_.position[j];
        }
    }

    for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
    {
        (void)command_interfaces_[i].set_value(current_positions_[i]);
    }

    return controller_interface::return_type::OK;
}

}  // namespace streaming_position_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    streaming_position_controller::StreamingPositionController,
    controller_interface::ControllerInterface)
