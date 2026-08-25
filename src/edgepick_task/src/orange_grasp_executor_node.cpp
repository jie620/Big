#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "edgepick_task/grasp_target_builder.hpp"
#include "edgepick_task/task_event_io.hpp"

namespace edgepick_task
{
namespace
{

using GripperCommand = control_msgs::action::GripperCommand;

class OrangeGraspExecutorNode final : public rclcpp::Node
{
public:
  OrangeGraspExecutorNode()
  : Node("edgepick_orange_grasp_executor")
  {
    move_group_name_ = declare_parameter<std::string>("move_group_name", "arm_group");
    event_topic_ = declare_parameter<std::string>("event_topic", "/edgepick/task/event");
    state_topic_ = declare_parameter<std::string>("state_topic", "/edgepick/task/state");
    pregrasp_pose_topic_ =
      declare_parameter<std::string>("pregrasp_pose_topic", "/edgepick/task/pregrasp_pose");
    grasp_pose_topic_ =
      declare_parameter<std::string>("grasp_pose_topic", "/edgepick/task/grasp_pose");
    gripper_action_name_ =
      declare_parameter<std::string>("gripper_action_name", "/grip_group_controller/gripper_cmd");
    startup_ready_topic_ =
      declare_parameter<std::string>("startup_ready_topic", "/edgepick/startup_pose/ready");
    require_startup_pose_ready_ =
      declare_parameter<bool>("require_startup_pose_ready", true);
    startup_pose_ready_ = !require_startup_pose_ready_;

    planning_time_sec_ = std::max(0.1, declare_parameter<double>("planning_time_sec", 5.0));
    planning_attempts_ =
      std::max(1, static_cast<int>(declare_parameter<int>("planning_attempts", 10)));
    state_monitor_wait_sec_ =
      std::max(0.1, declare_parameter<double>("state_monitor_wait_sec", 2.0));
    state_transition_wait_sec_ =
      std::max(0.1, declare_parameter<double>("state_transition_wait_sec", 10.0));
    startup_wait_sec_ = std::max(0.1, declare_parameter<double>("startup_wait_sec", 30.0));
    settle_time_ms_ = std::max(0, static_cast<int>(declare_parameter<int>("settle_time_ms", 500)));
    verification_settle_ms_ =
      std::max(0, static_cast<int>(declare_parameter<int>("verification_settle_ms", 300)));
    goal_position_tolerance_m_ =
      std::max(0.0, declare_parameter<double>("goal_position_tolerance_m", 0.01));
    goal_orientation_tolerance_rad_ =
      std::max(0.0, declare_parameter<double>("goal_orientation_tolerance_rad", 0.05));
    velocity_scaling_ = clamp_scaling(declare_parameter<double>("velocity_scaling_factor", 0.1));
    acceleration_scaling_ =
      clamp_scaling(declare_parameter<double>("acceleration_scaling_factor", 0.1));
    gripper_open_position_ = declare_parameter<double>("gripper_open_position", -0.0796);
    gripper_close_position_ = declare_parameter<double>("gripper_close_position", -1.4939);
    gripper_max_effort_ = std::max(0.0, declare_parameter<double>("gripper_max_effort", 0.0));

    event_publisher_ = create_publisher<std_msgs::msg::String>(event_topic_, 10);
    state_subscription_ = create_subscription<std_msgs::msg::String>(
      state_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr message) { handle_state_message(*message); });
    pregrasp_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pregrasp_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        handle_pose_message(*message, PoseSlot::kPregrasp);
      });
    grasp_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      grasp_pose_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        handle_pose_message(*message, PoseSlot::kGrasp);
      });
    startup_ready_subscription_ = create_subscription<std_msgs::msg::String>(
      startup_ready_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        if (message->data == "ready") {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            startup_pose_ready_ = true;
          }
          cv_.notify_all();
          RCLCPP_INFO(get_logger(), "Startup pose sequence is ready.");
        }
      });

    RCLCPP_INFO(
      get_logger(), "Orange grasp executor ready: group=%s gripper=%s plan_time=%.1f",
      move_group_name_.c_str(), gripper_action_name_.c_str(), planning_time_sec_);
  }

  void start()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), move_group_name_);
    configure_move_group();

    if (!move_group_->startStateMonitor(state_monitor_wait_sec_)) {
      throw std::runtime_error("MoveIt state monitor did not become ready");
    }

    planning_frame_ = move_group_->getPlanningFrame();
    gripper_client_ =
      rclcpp_action::create_client<GripperCommand>(this, gripper_action_name_);
    worker_ = std::thread([this]() { run(); });
  }

  ~OrangeGraspExecutorNode() override
  {
    stop_requested_ = true;
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  enum class PoseSlot
  {
    kPregrasp,
    kGrasp,
  };

  void configure_move_group()
  {
    move_group_->setPlanningTime(planning_time_sec_);
    move_group_->setNumPlanningAttempts(static_cast<unsigned int>(planning_attempts_));
    move_group_->allowReplanning(true);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->setGoalPositionTolerance(goal_position_tolerance_m_);
    move_group_->setGoalOrientationTolerance(goal_orientation_tolerance_rad_);
  }

  void handle_state_message(const std_msgs::msg::String & message)
  {
    const auto state = parse_task_state(message.data);
    if (!state.has_value()) {
      RCLCPP_WARN(get_logger(), "Ignoring unknown task state '%s'.", message.data.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_state_ = *state;
    }
    cv_.notify_all();
  }

  void handle_pose_message(const geometry_msgs::msg::PoseStamped & message, PoseSlot slot)
  {
    if (message.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring empty-frame grasp pose.");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (slot == PoseSlot::kPregrasp) {
        pregrasp_pose_ = message;
      } else {
        grasp_pose_ = message;
      }
    }
    cv_.notify_all();
  }

  void run()
  {
    struct ShutdownGuard
    {
      ~ShutdownGuard()
      {
        if (rclcpp::ok()) {
          rclcpp::shutdown();
        }
      }
    } shutdown_guard;

    bool succeeded = false;
    try {
      if (!wait_for_target_and_planning_state()) {
        if (current_state() == TaskState::kPlanning) {
          publish_event(TaskEvent::kTimeout);
        }
        return;
      }

      const auto target = capture_target();
      if (!target.has_value()) {
        publish_event(TaskEvent::kPlanFailed);
        return;
      }

      if (!move_to_pose(target->pregrasp_pose, "pregrasp")) {
        publish_event(TaskEvent::kPlanFailed);
        return;
      }

      publish_event(TaskEvent::kPlanSucceeded);
      if (!wait_for_state(TaskState::kExecuting)) {
        publish_event(TaskEvent::kTimeout);
        return;
      }

      if (!send_gripper_goal(gripper_open_position_, "open")) {
        publish_event(TaskEvent::kExecutionFailed);
        return;
      }
      settle(settle_time_ms_);

      if (!move_to_pose(target->grasp_pose, "grasp")) {
        publish_event(TaskEvent::kExecutionFailed);
        return;
      }

      if (!send_gripper_goal(gripper_close_position_, "close")) {
        publish_event(TaskEvent::kExecutionFailed);
        return;
      }

      if (!move_to_pose(target->pregrasp_pose, "retreat")) {
        publish_event(TaskEvent::kExecutionFailed);
        return;
      }

      publish_event(TaskEvent::kExecutionSucceeded);
      if (!wait_for_state(TaskState::kVerifying)) {
        publish_event(TaskEvent::kTimeout);
        return;
      }

      settle(verification_settle_ms_);
      publish_event(TaskEvent::kVerificationSucceeded);
      RCLCPP_INFO(get_logger(), "Orange grasp execution completed successfully.");
      succeeded = true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Orange grasp execution failed: %s", error.what());
    }

    exit_code_ = succeeded ? 0 : 1;
  }

  bool wait_for_target_and_planning_state()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto timeout = std::chrono::duration<double>(startup_wait_sec_);
    const bool ready = cv_.wait_for(lock, timeout, [this]() {
      return stop_requested_ ||
             (startup_pose_ready_ && current_state_ == TaskState::kPlanning &&
              pregrasp_pose_.has_value() &&
              grasp_pose_.has_value());
    });
    if (!ready || stop_requested_) {
      RCLCPP_ERROR(
        get_logger(),
        "Timed out waiting for planning state and grasp poses. Current state=%s",
        to_string(current_state_));
      return false;
    }
    return true;
  }

  std::optional<GraspTargetPoses> capture_target()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pregrasp_pose_.has_value() || !grasp_pose_.has_value()) {
      return std::nullopt;
    }
    if (pregrasp_pose_->header.frame_id != grasp_pose_->header.frame_id) {
      RCLCPP_ERROR(
        get_logger(),
        "Pregrasp frame '%s' does not match grasp frame '%s'.",
        pregrasp_pose_->header.frame_id.c_str(), grasp_pose_->header.frame_id.c_str());
      return std::nullopt;
    }
    if (!planning_frame_.empty() && pregrasp_pose_->header.frame_id != planning_frame_) {
      RCLCPP_WARN(
        get_logger(),
        "Target frame '%s' does not match MoveIt planning frame '%s'.",
        pregrasp_pose_->header.frame_id.c_str(), planning_frame_.c_str());
    }
    GraspTargetPoses poses;
    poses.pregrasp_pose = *pregrasp_pose_;
    poses.grasp_pose = *grasp_pose_;
    return poses;
  }

  TaskState current_state()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_state_;
  }

  bool wait_for_state(TaskState expected_state)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto timeout = std::chrono::duration<double>(state_transition_wait_sec_);
    const bool reached = cv_.wait_for(lock, timeout, [this, expected_state]() {
      return stop_requested_ || current_state_ == expected_state;
    });
    if (!reached || stop_requested_) {
      RCLCPP_ERROR(
        get_logger(), "Timed out waiting for task state %s; current=%s", to_string(expected_state),
        to_string(current_state_));
      return false;
    }
    return true;
  }

  bool move_to_pose(const geometry_msgs::msg::PoseStamped & pose, const std::string & label)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(), "Orange grasp %s planning failed with code %d.", label.c_str(),
        plan_result.val);
      return false;
    }

    const auto execute_result = move_group_->execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        get_logger(), "Orange grasp %s execution failed with code %d.", label.c_str(),
        execute_result.val);
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Orange grasp %s succeeded: frame=%s x=%.3f y=%.3f z=%.3f", label.c_str(),
      pose.header.frame_id.c_str(), pose.pose.position.x, pose.pose.position.y,
      pose.pose.position.z);
    return true;
  }

  bool send_gripper_goal(double position, const std::string & label)
  {
    const auto timeout = std::chrono::duration<double>(state_transition_wait_sec_);
    if (!gripper_client_->wait_for_action_server(timeout)) {
      RCLCPP_ERROR(
        get_logger(), "Gripper action server '%s' is not available.", gripper_action_name_.c_str());
      return false;
    }

    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = gripper_max_effort_;

    auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timed out sending gripper %s goal.", label.c_str());
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Gripper %s goal was rejected.", label.c_str());
      return false;
    }

    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for gripper %s result.", label.c_str());
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(
        get_logger(), "Gripper %s goal finished with code %d.", label.c_str(),
        static_cast<int>(wrapped_result.code));
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Gripper %s goal reached position %.4f.", label.c_str(), position);
    return true;
  }

  void publish_event(TaskEvent event)
  {
    std_msgs::msg::String message;
    message.data = to_string(event);
    event_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Published task event '%s'.", message.data.c_str());
  }

  void settle(int settle_ms) const
  {
    if (settle_ms > 0) {
      rclcpp::sleep_for(std::chrono::milliseconds(settle_ms));
    }
  }

  static double clamp_scaling(double value)
  {
    if (!std::isfinite(value)) {
      return 0.1;
    }
    return std::clamp(value, 0.01, 1.0);
  }

  std::string move_group_name_;
  std::string event_topic_;
  std::string state_topic_;
  std::string pregrasp_pose_topic_;
  std::string grasp_pose_topic_;
  std::string gripper_action_name_;
  std::string startup_ready_topic_;
  bool require_startup_pose_ready_{true};
  double planning_time_sec_{5.0};
  int planning_attempts_{10};
  double state_monitor_wait_sec_{2.0};
  double state_transition_wait_sec_{10.0};
  double startup_wait_sec_{30.0};
  int settle_time_ms_{500};
  int verification_settle_ms_{300};
  double goal_position_tolerance_m_{0.01};
  double goal_orientation_tolerance_rad_{0.05};
  double velocity_scaling_{0.1};
  double acceleration_scaling_{0.1};
  double gripper_open_position_{-0.0796};
  double gripper_close_position_{-1.4939};
  double gripper_max_effort_{0.0};
  std::string planning_frame_;
  std::optional<geometry_msgs::msg::PoseStamped> pregrasp_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> grasp_pose_;
  TaskState current_state_{TaskState::kIdle};
  bool startup_pose_ready_{true};
  std::atomic<int> exit_code_{1};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic_bool stop_requested_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pregrasp_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr startup_ready_subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  std::thread worker_;

public:
  int exit_code() const
  {
    return exit_code_.load();
  }
};

}  // namespace
}  // namespace edgepick_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<edgepick_task::OrangeGraspExecutorNode>();
  try {
    node->start();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Failed to start grasp executor: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return node->exit_code();
}
