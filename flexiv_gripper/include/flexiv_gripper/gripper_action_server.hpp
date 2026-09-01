/**
 * @file gripper_action_server.hpp
 * @brief Header file for GripperActionServer class
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_GRIPPER__GRIPPER_ACTION_SERVER_HPP_
#define FLEXIV_GRIPPER__GRIPPER_ACTION_SERVER_HPP_

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

// ROS
#include "control_msgs/action/gripper_command.hpp"
#include "flexiv_msgs/action/grasp.hpp"
#include "flexiv_msgs/action/move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

// Flexiv
#include "flexiv/rdk/data.hpp"
#include "flexiv/rdk/gripper.hpp"
#include "flexiv/rdk/robot.hpp"
#include "flexiv/rdk/tool.hpp"

#include "flexiv_hardware/fault_recovery.hpp"

namespace {

const int kDefaultStatePublishRate = 30;    // [Hz]
const int kDefaultFeedbackPublishRate = 10; // [Hz]
const double kDefaultVelocity = 0.1;        // [m/s]
const double kDefaultMaxForce = 20;         // [N]
// Tight tolerance used to short-circuit a Move() when the gripper is already at the requested
// width.
const double kGripperWidthTolerance = 2e-3;    // [m] (2 mm)
// Position tolerance for the action result on a close command
const double kReachedGoalTolerance = 5e-3;     // [m] (5 mm)
// Position tolerance for the action result on an open command
const double kOpenReachedGoalTolerance = 2e-2; // [m] (20 mm)
// Minimum absolute force that can count as a stall / object contact [N].
const double kStallForceThreshold = 0.5;       // [N]
// Fraction of commanded max_effort at which a close is considered stalled. Using only
// kStallForceThreshold (0.5 N) aborts Move() before the gripper can apply the requested grasp
// force (e.g. 80 N).
const double kStallForceFraction = 0.8;

// Bounded wait for the robot to become operational when using a normal RDK instance.
const std::chrono::seconds kOperationalTimeout {30};
}

namespace flexiv_gripper {

/**
 * @brief Check if the result of the asynchronous command is ready.
 * @tparam T The return type of the future.
 * @param[in] result_future The future object to check.
 * @param[in] timeout Future wait timeout.
 * @return True if the function has finished, false otherwise.
 */
template <typename T>
bool IsResultReady(std::future<T>& result_future, std::chrono::nanoseconds timeout)
{
    return result_future.wait_for(timeout) == std::future_status::ready;
}

class GripperActionServer : public rclcpp::Node
{

public:
    using Grasp = flexiv_msgs::action::Grasp;
    using GoalHandleGrasp = rclcpp_action::ServerGoalHandle<Grasp>;

    using Move = flexiv_msgs::action::Move;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

    using Trigger = std_srvs::srv::Trigger;

    explicit GripperActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // gripper actions
    enum class GripperAction
    {
        kGrasp,
        kMove,
        kGripperCommand
    };

    // return string of the gripper action
    static std::string GetGripperActionName(GripperAction action)
    {
        switch (action) {
            case GripperAction::kGrasp:
                return {"Grasping"};
            case GripperAction::kMove:
                return {"Moving"};
            case GripperAction::kGripperCommand:
                return {"GripperCommand"};
            default:
                throw std::invalid_argument("Invalid gripper action");
        }
    };

    // Flexiv RDK
    std::unique_ptr<flexiv::rdk::Robot> robot_;
    std::unique_ptr<flexiv::rdk::Gripper> gripper_;
    std::unique_ptr<flexiv::rdk::Tool> tool_;

    rclcpp_action::Server<Grasp>::SharedPtr grasp_action_server_;
    rclcpp_action::Server<Move>::SharedPtr move_action_server_;
    rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_action_server_;
    rclcpp::Service<Trigger>::SharedPtr stop_service_;
    rclcpp::TimerBase::SharedPtr state_publish_timer_;

    std::mutex gripper_states_mutex_;
    flexiv::rdk::GripperStates current_gripper_states_;

    double default_velocity_;
    double default_max_force_;
    std::chrono::nanoseconds future_wait_timeout_ {0};

    // Gripper joint states publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_ready_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_joint_states_publisher_;
    std::vector<std::string> gripper_joint_names_;

    /**
     * @brief Publish the current gripper states.
     */
    void PublishGripperStates();

    /**
     * @brief Stop the gripper.
     * @param[out] response Success or failure of the service call.
     */
    void StopServiceCallback(const std::shared_ptr<Trigger::Response>& response);

    /**
     * @brief Handle the gripper action cancel request.
     * @param[in] action Gripper action to cancel.
     */
    rclcpp_action::CancelResponse HandleCancel(GripperAction action);

    /**
     * @brief Handle the gripper action goal request.
     * @param[in] action Gripper action to handle.
     */
    rclcpp_action::GoalResponse HandleGoal(GripperAction action);

    /**
     * @brief Perform the gripper move action.
     */
    void ExecuteMove(const std::shared_ptr<GoalHandleMove>& goal_handle);

    /**
     * @brief Perform the gripper grasp action.
     */
    void ExecuteGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle);

    /**
     * @brief Perform the gripper command action.
     * @param[in] goal_handle The goal handle of the action.
     */
    void ExecuteGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle);

    /**
     * @brief Drive the GripperCommand action through completion. Dispatches the non-blocking RDK
     * Move(), blocks until motion finishes, then populates the action result from a fresh gripper
     * sample.
     * @param[in] goal_handle The goal handle of the action.
     * @param[in] target_width Commanded gripper width in meters.
     * @param[in] max_force Maximum force the firmware may apply.
     */
    void ExecuteGripperCommandHelper(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
        double target_width, double max_force);

    /**
     * @brief [Blocking] Wait until the RDK reports the gripper has finished moving, stalled on an
     * object, or the motion timeout elapsed. The RDK's Move() is non-blocking; this provides the
     * synchronous "motion complete" semantics ROS 2 action clients expect.
     * @param[in] target_width Commanded gripper width in meters.
     * @param[in] velocity Commanded velocity in m/s, used to estimate the minimum time the move
     * should take.
     * @param[in] max_force Upper bound on the force the firmware may apply. The effective stall
     * threshold is max(kStallForceThreshold, kStallForceFraction * max_force).
     */
    void WaitForGripperMotionComplete(double target_width, double velocity, double max_force);

    /**
     * @brief Resolve the effective max force for a GripperCommand goal. Goals with
     * max_effort <= 0 fall back to kDefaultMaxForce.
     */
    static double EffectiveMaxForce(double command_max_effort);

    /**
     * @brief Execute the gripper command and return the result.
     * @tparam T Gripper action message type (Grasp or Move).
     * @param[in] goal_handle The goal handle of the action.
     * @param[in] action The gripper action to execute.
     * @param[in] command The RDK function to execute the gripper command.
     */
    template <typename T>
    void ExecuteCommand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle,
        GripperAction action, const std::function<void()>& command)
    {
        const auto action_name = GetGripperActionName(action);
        RCLCPP_INFO(this->get_logger(), "Gripper %s action has been received", action_name.c_str());

        auto command_execution_result = CommandExecutionResult<T>(command);

        std::future<std::shared_ptr<typename T::Result>> result_future
            = std::async(std::launch::async, command_execution_result);

        while (!IsResultReady(result_future, future_wait_timeout_) && rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                gripper_->Stop();
                auto result = result_future.get();
                RCLCPP_INFO(
                    this->get_logger(), "Gripper %s action has been canceled", action_name.c_str());
                goal_handle->canceled(result);
                return;
            }
            PublishGripperStatesFeedback(goal_handle);
        }

        if (rclcpp::ok()) {
            const auto result = result_future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "Gripper %s action has been completed",
                    action_name.c_str());
                goal_handle->succeed(result);
            } else {
                RCLCPP_ERROR(
                    this->get_logger(), "Gripper %s action has failed", action_name.c_str());
                goal_handle->abort(result);
            }
        }
    }

    /**
     * @brief Return the command execution result.
     * @tparam T Gripper action message type (Grasp or Move).
     * @param[in] command  The function to execute the gripper command.
     * @return Success or failure of the command execution.
     */
    template <typename T>
    std::function<std::shared_ptr<typename T::Result>()> CommandExecutionResult(
        const std::function<void()>& command)
    {
        return std::function<std::shared_ptr<typename T::Result>()>([command]() {
            auto result = std::make_shared<typename T::Result>();
            try {
                command();
                result->success = true;
            } catch (const std::exception& e) {
                result->success = false;
                result->error = e.what();
            }
            return result;
        });
    }

    /**
     * @brief Publish the gripper states feedback in the action server.
     * @tparam T Gripper action message type (Grasp or Move).
     * @param[in] goal_handle The goal handle of the action.
     */
    template <typename T>
    void PublishGripperStatesFeedback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle)
    {
        auto feedback = std::make_shared<typename T::Feedback>();
        std::lock_guard<std::mutex> lock(gripper_states_mutex_);
        feedback->current_width = current_gripper_states_.width;
        feedback->current_force = current_gripper_states_.force;
        feedback->moving = current_gripper_states_.is_moving;
        goal_handle->publish_feedback(feedback);
    }

    /**
     * @brief Publish the gripper command feedback in the action server.
     * @param[in] goal_handle The goal handle of the action.
     */
    void PublishGripperCommandFeedback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle);
};

} // namespace flexiv_gripper

#endif /* FLEXIV_GRIPPER__GRIPPER_ACTION_SERVER_HPP_ */
