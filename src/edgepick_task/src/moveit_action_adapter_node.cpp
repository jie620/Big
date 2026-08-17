#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "edgepick_task/moveit_action_event_mapper.hpp"
#include "edgepick_task/task_event_io.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace edgepick_task
{
namespace
{

class MoveItActionAdapterNode final : public rclcpp::Node
{
public:
  MoveItActionAdapterNode()
  : Node("edgepick_moveit_action_adapter")
  {
    const std::string event_topic =
      declare_parameter<std::string>("event_topic", "/edgepick/task/event");
    const std::string state_topic =
      declare_parameter<std::string>("state_topic", "/edgepick/task/state");
    const std::string move_group_action_name =
      declare_parameter<std::string>("move_group_action_name", "/move_action");
    const std::string execute_action_name =
      declare_parameter<std::string>("execute_trajectory_action_name", "/execute_trajectory");
    const std::string planning_outcome_name =
      declare_parameter<std::string>("planning_outcome", "success");
    const std::string execution_outcome_name =
      declare_parameter<std::string>("execution_outcome", "success");

    use_mock_action_results_ = declare_parameter<bool>("use_mock_action_results", true);
    action_result_delay_ticks_ = delay_ticks_from_parameters();
    planning_outcome_ = parse_or_default(planning_outcome_name, MoveItActionOutcome::kSucceeded);
    execution_outcome_ = parse_or_default(execution_outcome_name, MoveItActionOutcome::kSucceeded);

    event_publisher_ = create_publisher<std_msgs::msg::String>(event_topic, 10);
    state_subscription_ = create_subscription<std_msgs::msg::String>(
      state_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr message) { handle_state_message(*message); });

    // Stage 7 creates typed MoveIt action clients and checks action-server
    // availability, but keeps goal construction disabled by default. That gives
    // us a safe adapter seam before we define target pose and trajectory policy.
    move_group_client_ =
      rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(this, move_group_action_name);
    execute_client_ = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
      this, execute_action_name);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(timer_period_ms_), [this]() { publish_result_when_ready(); });

    RCLCPP_INFO(
      get_logger(),
      "MoveIt action adapter ready: mock_results=%s planning_outcome=%s execution_outcome=%s",
      use_mock_action_results_ ? "true" : "false", to_string(planning_outcome_),
      to_string(execution_outcome_));
  }

private:
  std::size_t delay_ticks_from_parameters()
  {
    const int action_result_delay_ms =
      std::max(0, static_cast<int>(declare_parameter<int>("action_result_delay_ms", 300)));
    timer_period_ms_ =
      std::max(10, static_cast<int>(declare_parameter<int>("timer_period_ms", 100)));
    server_wait_ms_ =
      std::max(0, static_cast<int>(declare_parameter<int>("server_wait_ms", 100)));
    return static_cast<std::size_t>(action_result_delay_ms / timer_period_ms_);
  }

  MoveItActionOutcome parse_or_default(
    const std::string & value,
    MoveItActionOutcome fallback) const
  {
    const auto parsed = parse_moveit_action_outcome(value);
    if (!parsed.has_value()) {
      RCLCPP_WARN(
        get_logger(), "Unknown MoveIt action outcome '%s'; using %s.", value.c_str(),
        to_string(fallback));
      return fallback;
    }
    return *parsed;
  }

  void handle_state_message(const std_msgs::msg::String & message)
  {
    const auto state = parse_task_state(message.data);
    if (!state.has_value()) {
      RCLCPP_WARN(get_logger(), "Ignoring unknown task state '%s'.", message.data.c_str());
      return;
    }

    if (!last_state_.has_value() || *state != *last_state_) {
      last_state_ = *state;
      arm_stage_if_needed(*state);
    }
  }

  void arm_stage_if_needed(TaskState state)
  {
    if (state == TaskState::kPlanning) {
      pending_phase_ = MoveItTaskPhase::kPlanning;
      remaining_delay_ticks_ = action_result_delay_ticks_;
      RCLCPP_INFO(get_logger(), "Planning state observed; waiting for MoveGroup result.");
      return;
    }
    if (state == TaskState::kExecuting) {
      pending_phase_ = MoveItTaskPhase::kExecution;
      remaining_delay_ticks_ = action_result_delay_ticks_;
      RCLCPP_INFO(get_logger(), "Executing state observed; waiting for ExecuteTrajectory result.");
      return;
    }
    pending_phase_.reset();
  }

  void publish_result_when_ready()
  {
    if (!pending_phase_.has_value()) {
      return;
    }
    if (remaining_delay_ticks_ > 0) {
      --remaining_delay_ticks_;
      return;
    }

    const MoveItTaskPhase phase = *pending_phase_;
    pending_phase_.reset();

    const MoveItActionOutcome outcome = action_outcome_for_phase(phase);
    const TaskEvent event = task_event_for_moveit_outcome(phase, outcome);

    std_msgs::msg::String message;
    message.data = to_string(event);
    event_publisher_->publish(message);
    RCLCPP_INFO(
      get_logger(), "phase=%s action_outcome=%s published_event=%s", to_string(phase),
      to_string(outcome), message.data.c_str());
  }

  MoveItActionOutcome action_outcome_for_phase(MoveItTaskPhase phase)
  {
    if (use_mock_action_results_) {
      return configured_outcome_for_phase(phase);
    }

    if (!action_server_available(phase)) {
      return MoveItActionOutcome::kUnavailable;
    }

    RCLCPP_WARN(
      get_logger(),
      "MoveIt action server is available, but Stage 7 still uses configured dry-run outcomes; "
      "real goal construction is a later step.");
    return configured_outcome_for_phase(phase);
  }

  MoveItActionOutcome configured_outcome_for_phase(MoveItTaskPhase phase) const
  {
    return phase == MoveItTaskPhase::kPlanning ? planning_outcome_ : execution_outcome_;
  }

  bool action_server_available(MoveItTaskPhase phase)
  {
    const auto timeout = std::chrono::milliseconds(server_wait_ms_);
    if (phase == MoveItTaskPhase::kPlanning) {
      return move_group_client_->wait_for_action_server(timeout);
    }
    return execute_client_->wait_for_action_server(timeout);
  }

  bool use_mock_action_results_{true};
  int timer_period_ms_{100};
  int server_wait_ms_{100};
  std::size_t action_result_delay_ticks_{0};
  std::size_t remaining_delay_ticks_{0};
  MoveItActionOutcome planning_outcome_{MoveItActionOutcome::kSucceeded};
  MoveItActionOutcome execution_outcome_{MoveItActionOutcome::kSucceeded};
  std::optional<TaskState> last_state_;
  std::optional<MoveItTaskPhase> pending_phase_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
  rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SharedPtr execute_client_;
};

}  // namespace
}  // namespace edgepick_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgepick_task::MoveItActionAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
