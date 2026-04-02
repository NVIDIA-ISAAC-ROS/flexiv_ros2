/**
 * @file streaming_position_controller.hpp
 * @brief Minimal ros2_control controller that accepts sensor_msgs/JointState
 *        on a topic and writes positions directly to the hardware command
 *        interface.
 */

#ifndef STREAMING_POSITION_CONTROLLER__STREAMING_POSITION_CONTROLLER_HPP_
#define STREAMING_POSITION_CONTROLLER__STREAMING_POSITION_CONTROLLER_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace streaming_position_controller {

class StreamingPositionController : public controller_interface::ControllerInterface
{
public:
    StreamingPositionController();
    ~StreamingPositionController() override = default;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::vector<std::string> joint_names_;
    std::vector<std::string> command_interface_types_;
    std::unordered_map<std::string, std::size_t> name_to_index_;

    realtime_tools::RealtimeThreadSafeBox<sensor_msgs::msg::JointState> rt_command_;
    sensor_msgs::msg::JointState last_command_;

    std::vector<std::size_t> pos_state_idx_;
    std::vector<double> current_positions_;
    bool positions_initialized_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscriber_;
};

}  // namespace streaming_position_controller

#endif  // STREAMING_POSITION_CONTROLLER__STREAMING_POSITION_CONTROLLER_HPP_
