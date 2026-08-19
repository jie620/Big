#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "edgepick_interfaces/msg/target_detection_array.hpp"
#include "edgepick_perception/perception_metrics.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace edgepick_perception
{
namespace
{

std::optional<double> stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec == 0 && stamp.nanosec == 0U) {
    return std::nullopt;
  }
  return static_cast<double>(stamp.sec) + (static_cast<double>(stamp.nanosec) * 1.0e-9);
}

std::optional<double> best_detection_score(
  const edgepick_interfaces::msg::TargetDetectionArray & message)
{
  std::optional<double> best_score;
  for (const auto & detection : message.detections) {
    const double score = static_cast<double>(detection.score);
    if (!std::isfinite(score)) {
      continue;
    }
    if (!best_score.has_value() || score > *best_score) {
      best_score = score;
    }
  }
  return best_score;
}

class PerceptionMetricsNode final : public rclcpp::Node
{
public:
  PerceptionMetricsNode()
  : Node("edgepick_perception_metrics")
  {
    const std::string detections_topic =
      declare_parameter<std::string>("detections_topic", "/edgepick/perception/detections");
    const std::string target_topic =
      declare_parameter<std::string>("target_topic", "/edgepick/perception/target_point");
    const std::string event_topic =
      declare_parameter<std::string>("event_topic", "/edgepick/task/event");
    const std::string metrics_topic =
      declare_parameter<std::string>("metrics_topic", "/edgepick/perception/metrics");
    const int publish_period_ms =
      std::max(100, static_cast<int>(declare_parameter<int>("publish_period_ms", 5000)));

    metrics_publisher_ = create_publisher<std_msgs::msg::String>(metrics_topic, 10);
    detections_subscription_ =
      create_subscription<edgepick_interfaces::msg::TargetDetectionArray>(
      detections_topic, 10,
      [this](const edgepick_interfaces::msg::TargetDetectionArray::SharedPtr message) {
        handle_detections(*message);
      });
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic, 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
        handle_target_point(*message);
      });
    event_subscription_ = create_subscription<std_msgs::msg::String>(
      event_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr message) { handle_task_event(*message); });
    timer_ = create_wall_timer(
      std::chrono::milliseconds{publish_period_ms}, [this]() { publish_summary(); });

    RCLCPP_INFO(
      get_logger(),
      "Perception metrics watching detections='%s' target='%s' events='%s'; publishing '%s'.",
      detections_topic.c_str(), target_topic.c_str(), event_topic.c_str(), metrics_topic.c_str());
  }

private:
  void handle_detections(const edgepick_interfaces::msg::TargetDetectionArray & message)
  {
    metrics_.record_detection_message(
      stamp_seconds(message.header.stamp), now().seconds(), message.detections.size(),
      best_detection_score(message));
  }

  void handle_target_point(const geometry_msgs::msg::PointStamped & message)
  {
    metrics_.record_target_point(
      stamp_seconds(message.header.stamp), now().seconds(), message.point.x, message.point.y,
      message.point.z);
  }

  void handle_task_event(const std_msgs::msg::String & message)
  {
    metrics_.record_task_event(message.data);
  }

  void publish_summary()
  {
    std_msgs::msg::String message;
    message.data = format_metrics_summary(metrics_.snapshot());
    metrics_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", message.data.c_str());
  }

  PerceptionMetricsAccumulator metrics_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_publisher_;
  rclcpp::Subscription<edgepick_interfaces::msg::TargetDetectionArray>::SharedPtr
    detections_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace edgepick_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgepick_perception::PerceptionMetricsNode>());
  rclcpp::shutdown();
  return 0;
}
