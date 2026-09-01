#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

#include "flexiv_gripper/gripper_action_server.hpp"

namespace flexiv_gripper {

GripperActionServer::GripperActionServer(const rclcpp::NodeOptions& options)
: Node("flexiv_gripper_node", options)
{
    this->declare_parameter("robot_sn", std::string());
    this->declare_parameter("gripper_name", std::string());
    this->declare_parameter("state_publish_rate", kDefaultStatePublishRate);
    this->declare_parameter("feedback_publish_rate", kDefaultFeedbackPublishRate);
    this->declare_parameter("default_velocity", kDefaultVelocity);
    this->declare_parameter("default_max_force", kDefaultMaxForce);
    this->declare_parameter("gripper_joint_names", std::vector<std::string>());
    this->declare_parameter("use_lite_rdk", false);

    std::string robot_sn;
    if (!this->get_parameter("robot_sn", robot_sn)) {
        RCLCPP_FATAL(this->get_logger(), "Parameter 'robot_sn' is not set");
        throw std::invalid_argument("Parameter 'robot_sn' is not set");
    }

    std::string gripper_name;
    if (!this->get_parameter("gripper_name", gripper_name)) {
        RCLCPP_FATAL(this->get_logger(), "Parameter 'gripper_name' is not set");
        throw std::invalid_argument("Parameter 'gripper_name' is not set");
    }

    this->default_velocity_ = this->get_parameter("default_velocity").as_double();
    this->default_max_force_ = this->get_parameter("default_max_force").as_double();

    if (!this->get_parameter("gripper_joint_names", this->gripper_joint_names_)) {
        RCLCPP_WARN(this->get_logger(), "Parameter 'gripper_joint_names' is not set");
        this->gripper_joint_names_ = {""};
    }

    const bool use_lite_rdk = this->get_parameter("use_lite_rdk").as_bool();
    const double kStatePublishRate
        = static_cast<double>(this->get_parameter("state_publish_rate").as_int());
    const double kFeedbackPublishRate
        = static_cast<double>(this->get_parameter("feedback_publish_rate").as_int());
    this->future_wait_timeout_ = rclcpp::WallRate(kFeedbackPublishRate).period();
    this->gripper_ready_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "~/ready", rclcpp::QoS(1).reliable().transient_local());

    try {
        RCLCPP_INFO(this->get_logger(), "Connecting to robot %s with a %s RDK instance ...",
            robot_sn.c_str(), use_lite_rdk ? "lite" : "normal");
        robot_ = std::make_unique<flexiv::rdk::Robot>(
            robot_sn, std::vector<std::string> {}, true, use_lite_rdk);

        RCLCPP_INFO(this->get_logger(), "Successfully connected to robot");

        if (!use_lite_rdk) {
            if (robot_->fault()) {
                RCLCPP_WARN(
                    this->get_logger(), "Fault occurred on robot server, trying to clear ...");
                if (!robot_->ClearFault()) {
                    RCLCPP_FATAL(get_logger(), "Fault cannot be cleared, exiting ...");
                    throw std::runtime_error("Fault cannot be cleared");
                }
                RCLCPP_INFO(this->get_logger(), "Fault on robot server is cleared");
            }

            if (!robot_->operational()) {
                // Enable() throws if the E-stop is not released, so report the real cause first.
                if (!robot_->estop_released()) {
                    throw std::runtime_error(flexiv_hardware::DescribeRobotCondition(
                        {robot_->connected(), robot_->operational_status(), false}));
                }

                RCLCPP_INFO(this->get_logger(), "Enabling robot ...");
                robot_->Enable();

                // Bounded, so that a robot that never becomes ready fails the node startup with
                // an actionable message instead of hanging in the constructor forever.
                const auto deadline = std::chrono::steady_clock::now() + kOperationalTimeout;
                while (!robot_->operational()) {
                    if (std::chrono::steady_clock::now() >= deadline) {
                        throw std::runtime_error(
                            "Robot did not become operational within "
                            + std::to_string(kOperationalTimeout.count()) + " s. "
                            + flexiv_hardware::DescribeRobotCondition(
                                {robot_->connected(), robot_->operational_status(), false}));
                    }
                    RCLCPP_INFO(this->get_logger(),
                        "Waiting for the robot to become operational: %s",
                        flexiv_hardware::OperationalStatusName(robot_->operational_status())
                            .c_str());
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                RCLCPP_INFO(this->get_logger(), "Robot is now operational");
            }
        }

        RCLCPP_INFO(this->get_logger(), "Initializing Flexiv gripper control interface");
        this->gripper_ = std::make_unique<flexiv::rdk::Gripper>(*robot_);
        this->tool_ = std::make_unique<flexiv::rdk::Tool>(*robot_);

        // Enable the specified gripper as a device
        RCLCPP_INFO(this->get_logger(), "Enabling gripper %s ...", gripper_name.c_str());
        gripper_->Enable(gripper_name);

        // Switch robot tool to gripper so the gravity compensation and TCP location is updated
        RCLCPP_INFO(this->get_logger(), "Switching robot tool to %s ...", gripper_name.c_str());
        tool_->Switch(gripper_name);

        // Manually initialize the gripper, not all grippers need this step
        RCLCPP_INFO(
            this->get_logger(), "Initializing gripper, this process takes about 10 seconds ..");
        gripper_->Init();
        std::this_thread::sleep_for(std::chrono::seconds(10));
        RCLCPP_INFO(this->get_logger(), "Gripper initialization completed");

        // Get the current gripper states
        this->current_gripper_states_ = gripper_->states();
    } catch (const std::exception& e) {
        if (use_lite_rdk) {
            RCLCPP_FATAL(this->get_logger(),
                "Failed to start gripper with a lite RDK instance: %s. Ensure the robot driver "
                "is already running with a normal RDK connection, or relaunch the gripper with "
                "parameter 'use_lite_rdk:=false' for standalone operation.",
                e.what());
        } else {
            RCLCPP_FATAL(this->get_logger(), "%s", e.what());
        }
        throw;
    }

    // Create the stop service server
    this->stop_service_
        = create_service<Trigger>("~/stop", [this](std::shared_ptr<Trigger::Request> /*request*/,
                                                std::shared_ptr<Trigger::Response> response) {
              return StopServiceCallback(std::move(response));
          });

    // Create the action servers
    const auto kMoveAction = GripperAction::kMove;
    this->move_action_server_ = rclcpp_action::create_server<Move>(
        this, "~/move",
        [this, kMoveAction](auto /*uuid*/, auto /*goal*/) { return HandleGoal(kMoveAction); },
        [this, kMoveAction](const auto& /*goal_handle*/) { return HandleCancel(kMoveAction); },
        [this](const auto& goal_handle) {
            return std::thread {[this, goal_handle]() { ExecuteMove(goal_handle); }}.detach();
        });

    const auto kGraspAction = GripperAction::kGrasp;
    this->grasp_action_server_ = rclcpp_action::create_server<Grasp>(
        this, "~/grasp",
        [this, kGraspAction](auto /*uuid*/, auto /*goal*/) { return HandleGoal(kGraspAction); },
        [this, kGraspAction](const auto& /*goal_handle*/) { return HandleCancel(kGraspAction); },
        [this](const auto& goal_handle) {
            return std::thread {[this, goal_handle]() { ExecuteGrasp(goal_handle); }}.detach();
        });

    const auto kGripperCommandAction = GripperAction::kGripperCommand;
    this->gripper_command_action_server_ = rclcpp_action::create_server<GripperCommand>(
        this, "~/gripper_action",
        [this, kGripperCommandAction](
            auto /*uuid*/, auto /*goal*/) { return HandleGoal(kGripperCommandAction); },
        [this, kGripperCommandAction](
            const auto& /*goal_handle*/) { return HandleCancel(kGripperCommandAction); },
        [this](const auto& goal_handle) {
            return std::thread {[this, goal_handle]() {
                ExecuteGripperCommand(goal_handle);
            }}.detach();
        });

    this->gripper_joint_states_publisher_
        = this->create_publisher<sensor_msgs::msg::JointState>("~/gripper_joint_states", 1);
    this->state_publish_timer_ = this->create_wall_timer(
        rclcpp::WallRate(kStatePublishRate).period(), [this]() { return PublishGripperStates(); });

    auto ready_msg = std_msgs::msg::Bool();
    ready_msg.data = true;
    this->gripper_ready_publisher_->publish(ready_msg);
    RCLCPP_INFO(this->get_logger(), "Published gripper readiness on ~/ready");
}

rclcpp_action::CancelResponse GripperActionServer::HandleCancel(GripperAction action)
{
    const auto action_name = GetGripperActionName(action);
    RCLCPP_INFO(this->get_logger(), "Canceling %s action", action_name.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse GripperActionServer::HandleGoal(GripperAction action)
{
    const auto action_name = GetGripperActionName(action);
    RCLCPP_INFO(this->get_logger(), "Received %s action request", action_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void GripperActionServer::ExecuteMove(const std::shared_ptr<GoalHandleMove>& goal_handle)
{
    auto command = [this, goal_handle]() {
        const auto goal = goal_handle->get_goal();
        gripper_->Move(goal->width, goal->velocity, goal->max_force);
    };
    ExecuteCommand(goal_handle, GripperAction::kMove, command);
}

void GripperActionServer::ExecuteGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle)
{
    auto command = [this, goal_handle]() {
        const auto goal = goal_handle->get_goal();
        gripper_->Grasp(goal->force);
    };
    ExecuteCommand(goal_handle, GripperAction::kGrasp, command);
}

double GripperActionServer::EffectiveMaxForce(double command_max_effort)
{
    return command_max_effort > 0.0 ? command_max_effort : kDefaultMaxForce;
}

void GripperActionServer::ExecuteGripperCommand(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle)
{
    const auto goal = goal_handle->get_goal();
    const double target_width = goal->command.position;
    const double max_force = EffectiveMaxForce(goal->command.max_effort);

    std::unique_lock<std::mutex> guard(gripper_states_mutex_);
    auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
    const double current_width = current_gripper_states_.width;
    if (target_width > gripper_->params().max_width || target_width < 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid gripper target width: %f. Max width = %f",
            target_width, gripper_->params().max_width);
        goal_handle->abort(result);
        return;
    }
    // Skip dispatch only when already essentially at the requested width
    if (std::abs(target_width - current_width) < kGripperWidthTolerance) {
        RCLCPP_INFO(this->get_logger(), "Gripper is already at the target width: %f", target_width);
        result->effort = current_gripper_states_.force;
        result->position = current_gripper_states_.width;
        result->reached_goal = true;
        result->stalled = false;
        goal_handle->succeed(result);
        return;
    }
    guard.unlock();

    ExecuteGripperCommandHelper(goal_handle, target_width, max_force);
}

void GripperActionServer::WaitForGripperMotionComplete(
    double target_width, double velocity, double max_force)
{
    constexpr auto kPollPeriod = std::chrono::milliseconds(20);
    constexpr auto kMotionTimeout = std::chrono::seconds(5);
    // Consecutive idle samples required before trusting motion has ended.
    // Filters the brief is_moving=false the firmware reports between
    // Move() returning and the underlying motion actually starting.
    constexpr int kIdleConfirmSamples = 3;
    // Extra time added to the analytical travel estimate to absorb
    // command-pipeline latency before we begin trusting the idle/stall
    // exit paths.
    constexpr double kMinWindowSlackSec = 0.15;

    const double stall_threshold = std::max(
        kStallForceThreshold, kStallForceFraction * max_force);
    // Guard against divide-by-zero in the min-window calculation if a
    // caller dispatched with a non-positive velocity.
    const double effective_velocity = (velocity > 0.0) ? velocity : kDefaultVelocity;

    // Minimum window the gripper needs to traverse the requested distance.
    // Used to gate the idle and stall exit paths so we don't return before
    // motion has had time to start.
    const double initial_width = gripper_->states().width;
    const double distance = std::abs(target_width - initial_width);
    const auto kMinMotionWindow = std::min(
        std::chrono::duration_cast<std::chrono::milliseconds>(kMotionTimeout),
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(distance / effective_velocity + kMinWindowSlackSec)));

    const auto start_time = std::chrono::steady_clock::now();
    const auto min_wait_until = start_time + kMinMotionWindow;

    int consecutive_idle_samples = 0;

    while (std::chrono::steady_clock::now() - start_time < kMotionTimeout) {
        const auto states = gripper_->states();

        // Reaching the target width is always conclusive, even within the
        // min window.
        if (std::abs(states.width - target_width) < kReachedGoalTolerance) {
            return;
        }

        const bool past_min_window
            = std::chrono::steady_clock::now() >= min_wait_until;

        // Force-stall and idle-exit are only trustworthy past the min
        // motion window. Inside the window, force may be residual from the
        // prior command (e.g. opening right after a grasp) and is_moving
        // may not yet have flipped to true.
        if (past_min_window && states.force >= stall_threshold) {
            // Halt the active drive so the firmware isn't still trying to
            // close further on the object after we hand control back.
            gripper_->Stop();
            return;
        }

        if (states.is_moving) {
            consecutive_idle_samples = 0;
        } else {
            ++consecutive_idle_samples;
            if (past_min_window && consecutive_idle_samples >= kIdleConfirmSamples) {
                return;
            }
        }

        std::this_thread::sleep_for(kPollPeriod);
    }

    RCLCPP_WARN(this->get_logger(),
        "Gripper motion did not finish within %lld ms (target=%f, last width=%f)",
        static_cast<long long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(kMotionTimeout).count()),
        target_width, gripper_->states().width);
}

void GripperActionServer::ExecuteGripperCommandHelper(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
    double target_width,
    double max_force)
{
    const auto action_name = GetGripperActionName(GripperAction::kGripperCommand);
    RCLCPP_INFO(this->get_logger(), "Gripper %s action has been received", action_name.c_str());

    // Snapshot velocity into a local so Move() and the wait use the same
    // value, and the lambda capture below stays by-value.
    const double velocity = default_velocity_;
    // Snapshot starting width for direction-aware result tolerance below.
    const double initial_width = gripper_->states().width;

    // RDK Move() is non-blocking; wait for motion to finish so the action
    // doesn't return success before the gripper has physically moved.
    auto run_command = [this, target_width, velocity, max_force]() {
        auto result = std::make_shared<GripperCommand::Result>();
        try {
            gripper_->Move(target_width, velocity, max_force);
            WaitForGripperMotionComplete(target_width, velocity, max_force);
            result->reached_goal = true;
        } catch (const std::exception& e) {
            result->reached_goal = false;
            RCLCPP_ERROR(this->get_logger(), "Gripper command failed: %s", e.what());
        }
        return result;
    };

    std::future<std::shared_ptr<typename GripperCommand::Result>> result_future
        = std::async(std::launch::async, run_command);

    while (!IsResultReady(result_future, future_wait_timeout_) && rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            gripper_->Stop();
            auto result = result_future.get();
            RCLCPP_INFO(
                this->get_logger(), "Gripper %s action has been canceled", action_name.c_str());
            goal_handle->canceled(result);
            return;
        }
        PublishGripperCommandFeedback(goal_handle);
    }

    if (!rclcpp::ok()) {
        return;
    }

    const auto result = result_future.get();
    // Fresh sample so the result reflects the gripper's actual final state
    // rather than whatever the publisher timer last cached.
    const auto final_states = gripper_->states();
    {
        std::lock_guard<std::mutex> guard(gripper_states_mutex_);
        current_gripper_states_ = final_states;
    }
    result->position = final_states.width;
    result->effort = final_states.force;

    // Direction-aware tolerance: opens may settle at the firmware's
    // natural rest position short of the target, while closes must stay
    // tight so a missed grasp (closed past the expected width with no
    // contact) still surfaces as a failure.
    const bool is_opening = (target_width > initial_width);
    const double reach_tolerance
        = is_opening ? kOpenReachedGoalTolerance : kReachedGoalTolerance;
    const bool reached_target
        = std::abs(final_states.width - target_width) < reach_tolerance;

    const double stall_threshold = std::max(
        kStallForceThreshold, kStallForceFraction * max_force);
    const bool stalled = final_states.force >= stall_threshold;

    // At this point result->reached_goal means "dispatch did not throw".
    // Combine with the position check so it reflects arrival too.
    result->reached_goal = result->reached_goal && reached_target;
    result->stalled = stalled;

    if (result->reached_goal || result->stalled) {
        RCLCPP_INFO(this->get_logger(),
            "Gripper %s action has been completed (target=%f, width=%f, force=%f, stalled=%s)",
            action_name.c_str(), target_width, final_states.width, final_states.force,
            result->stalled ? "true" : "false");
        goal_handle->succeed(result);
    } else {
        RCLCPP_ERROR(this->get_logger(),
            "Gripper %s action has failed (target=%f, width=%f, force=%f)", action_name.c_str(),
            target_width, final_states.width, final_states.force);
        goal_handle->abort(result);
    }
}

void GripperActionServer::StopServiceCallback(const std::shared_ptr<Trigger::Response>& response)
{
    RCLCPP_INFO(this->get_logger(), "Stopping the gripper...");
    auto result = CommandExecutionResult<Move>([this]() { gripper_->Stop(); })();
    response->success = result->success;
    response->message = result->error;
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Gripper has been stopped");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop the gripper");
    }
    if (!response->message.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Error message: %s", response->message.c_str());
    }
}

void GripperActionServer::PublishGripperStates()
{
    std::lock_guard<std::mutex> lock(gripper_states_mutex_);
    this->current_gripper_states_ = gripper_->states();
    // Modify the gripper joint states based on the mounted gripper type
    // The gripper joint states below is for the Flexiv Grav GN-01 gripper
    sensor_msgs::msg::JointState gripper_joint_states;
    gripper_joint_states.header.stamp = this->now();
    gripper_joint_states.name.push_back(this->gripper_joint_names_[0]);
    gripper_joint_states.position.push_back(this->current_gripper_states_.width);
    gripper_joint_states.velocity.push_back(0.0);
    gripper_joint_states.effort.push_back(this->current_gripper_states_.force);
    this->gripper_joint_states_publisher_->publish(gripper_joint_states);
}

void GripperActionServer::PublishGripperCommandFeedback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle)
{
    auto feedback = std::make_shared<GripperCommand::Feedback>();
    std::lock_guard<std::mutex> guard(gripper_states_mutex_);
    feedback->position = current_gripper_states_.width;
    feedback->effort = current_gripper_states_.force;
    goal_handle->publish_feedback(feedback);
}

} // namespace flexiv_gripper

RCLCPP_COMPONENTS_REGISTER_NODE(flexiv_gripper::GripperActionServer)
