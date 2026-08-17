#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "edgepick_task/mock_task_script.hpp"
#include "edgepick_task/task_event_io.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace edgepick_task
{
namespace
{

class MockTaskDriverNode final : public rclcpp::Node
{
public:
  MockTaskDriverNode()
  : Node("edgepick_mock_task_driver")
  {
    const std::string scenario = declare_parameter<std::string>("scenario", "success");
    const std::string event_topic =
      declare_parameter<std::string>("event_topic", "/edgepick/task/event");
    const std::string state_topic =
      declare_parameter<std::string>("state_topic", "/edgepick/task/state");
    const int event_period_ms =
      std::max(10, static_cast<int>(declare_parameter<int>("event_period_ms", 300)));
    const int initial_delay_ms =
      std::max(0, static_cast<int>(declare_parameter<int>("initial_delay_ms", 500)));
    warmup_ticks_ = static_cast<std::size_t>(initial_delay_ms / event_period_ms);
    script_ = std::make_unique<MockTaskScript>(make_mock_task_script(scenario));

    event_publisher_ = create_publisher<std_msgs::msg::String>(event_topic, 10);
    state_subscription_ = create_subscription<std_msgs::msg::String>(
      state_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr message) { handle_state_message(*message); });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(event_period_ms), [this]() { publish_next_when_ready(); });

    RCLCPP_INFO(
      get_logger(), "Loaded mock task scenario '%s' with %zu scripted events.",
      script_->name().c_str(), script_->step_count());
  }

private:
  void handle_state_message(const std_msgs::msg::String & message)
  {
    const auto state = parse_task_state(message.data);
    if (!state.has_value()) {
      RCLCPP_WARN(get_logger(), "Ignoring unknown task state '%s'.", message.data.c_str());
      return;
    }
    last_state_ = *state;
  }

  void publish_next_when_ready()
  {
    if (script_->complete()) {
      if (!completion_logged_) {
        RCLCPP_INFO(get_logger(), "Mock task scenario '%s' completed.", script_->name().c_str());
        completion_logged_ = true;
      }
      timer_->cancel();
      return;
    }

    if (warmup_ticks_ > 0) {
      --warmup_ticks_;
      return;
    }

    const MockTaskStep * step = script_->next_step();
    if (step == nullptr) {
      return;
    }

    // Bootstrap from idle even if the driver missed task_node's startup state
    // publication. Every later step waits for the state topic, so the scenario
    // still follows the real task-node transitions rather than blindly sleeping.
    std::optional<TaskState> observed_state = last_state_;
    if (!observed_state.has_value() && step->expected_state == TaskState::kIdle) {
      observed_state = TaskState::kIdle;
    }
    if (!observed_state.has_value() || *observed_state != step->expected_state) {
      return;
    }

    const auto event = script_->next_event_for_state(*observed_state);
    if (!event.has_value()) {
      return;
    }

    std_msgs::msg::String message;
    message.data = to_string(*event);
    event_publisher_->publish(message);
    RCLCPP_INFO(
      get_logger(), "adapter=%s observed_state=%s published_event=%s step=%zu/%zu",
      step->adapter_name.c_str(), to_string(*observed_state), message.data.c_str(),
      script_->cursor(), script_->step_count());
  }

  std::unique_ptr<MockTaskScript> script_;
  std::optional<TaskState> last_state_;
  std::size_t warmup_ticks_{0};
  bool completion_logged_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace edgepick_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgepick_task::MockTaskDriverNode>());
  rclcpp::shutdown();
  return 0;
}
