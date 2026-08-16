#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "edgepick_task/grasp_state_machine.hpp"
#include "edgepick_task/task_event_io.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace edgepick_task
{
namespace
{

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

class TaskNode final : public rclcpp::Node
{
public:
  TaskNode()
  : Node("edgepick_task_node"),
    machine_(TaskConfig{static_cast<std::size_t>(declare_parameter<int>(
      "max_recovery_attempts", 2))})
  {
    event_topic_ = declare_parameter<std::string>("event_topic", "/edgepick/task/event");
    state_topic_ = declare_parameter<std::string>("state_topic", "/edgepick/task/state");
    failure_topic_ = declare_parameter<std::string>("failure_topic", "/edgepick/task/failure");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/diagnostics");

    state_publisher_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);
    failure_publisher_ = create_publisher<std_msgs::msg::String>(failure_topic_, 10);
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
    event_subscription_ = create_subscription<std_msgs::msg::String>(
      event_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr message) { handle_event_message(*message); });

    publish_all("startup");
  }

private:
  void handle_event_message(const std_msgs::msg::String & message)
  {
    // Stage 5 keeps external adapters simple: perception, planning, execution,
    // and verification components only need to publish one stable event name.
    // Unknown strings are rejected at the ROS boundary and never enter the
    // state machine as "best effort" guesses.
    const auto event = parse_task_event(message.data);
    if (!event.has_value()) {
      RCLCPP_WARN(
        get_logger(), "Rejecting unknown task event '%s'.", message.data.c_str());
      publish_diagnostics("invalid_event", diagnostic_msgs::msg::DiagnosticStatus::WARN);
      return;
    }

    const TransitionResult result = machine_.handle(*event);
    RCLCPP_INFO(
      get_logger(), "event=%s status=%s state=%s->%s failure=%s recovery_attempts=%zu",
      to_string(*event), to_string(result.status), to_string(result.previous_state),
      to_string(result.current_state), to_string(result.failure), result.recovery_attempts);
    publish_all(to_string(*event), result.status);
  }

  void publish_all(
    const std::string & reason,
    TransitionStatus status = TransitionStatus::kAccepted)
  {
    // Always publish the three observability surfaces together so a rosbag or
    // terminal session can reconstruct state, failure cause, and diagnostic
    // severity from the same transition.
    publish_state();
    publish_failure();
    publish_diagnostics(reason, diagnostic_level(status));
  }

  void publish_state()
  {
    std_msgs::msg::String message;
    message.data = to_string(machine_.state());
    state_publisher_->publish(message);
  }

  void publish_failure()
  {
    std_msgs::msg::String message;
    message.data = to_string(machine_.last_failure());
    failure_publisher_->publish(message);
  }

  void publish_diagnostics(const std::string & reason, unsigned char level)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    // Diagnostics mirrors the task snapshot instead of embedding control logic.
    // This makes the node useful in mock tests now and compatible with a future
    // real MoveIt/I2C chain without changing the state vocabulary.
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "edgepick_task/state_machine";
    status.hardware_id = "edgepick_task";
    status.level = level;
    status.message = task_status_line(snapshot_from_machine(machine_));
    status.values.push_back(key_value("reason", reason));
    status.values.push_back(key_value("state", to_string(machine_.state())));
    status.values.push_back(key_value("failure", to_string(machine_.last_failure())));
    status.values.push_back(key_value("recovery_attempts", std::to_string(machine_.recovery_attempts())));
    status.values.push_back(key_value("terminal", machine_.is_terminal() ? "true" : "false"));

    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  unsigned char diagnostic_level(TransitionStatus status) const
  {
    if (machine_.state() == TaskState::kFailed) {
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    if (machine_.state() == TaskState::kRecovering || status == TransitionStatus::kRejected) {
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  GraspStateMachine machine_;
  std::string event_topic_;
  std::string state_topic_;
  std::string failure_topic_;
  std::string diagnostics_topic_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr failure_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_subscription_;
};

}  // namespace
}  // namespace edgepick_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgepick_task::TaskNode>());
  rclcpp::shutdown();
  return 0;
}
