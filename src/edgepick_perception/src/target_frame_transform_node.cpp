#include <chrono>
#include <memory>
#include <string>

#include "edgepick_perception/target_frame_transform.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace edgepick_perception
{
namespace
{

class TargetFrameTransformNode final : public rclcpp::Node
{
public:
  TargetFrameTransformNode()
  : Node("edgepick_target_frame_transform")
  {
    input_topic_ =
      declare_parameter<std::string>("input_topic", "/edgepick/perception/target_point");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/edgepick/perception/target_point_base");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    transform_timeout_ms_ =
      static_cast<int>(declare_parameter<int>("transform_timeout_ms", 100));

    target_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(output_topic_, 10);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      input_topic_, 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
        handle_target_point(*message);
      });

    RCLCPP_INFO(
      get_logger(), "Target frame transform watching '%s' and publishing '%s' in frame '%s'.",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
  }

private:
  void handle_target_point(const geometry_msgs::msg::PointStamped & message)
  {
    if (message.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target point has an empty source frame; skipping transform.");
      return;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame_, message.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(static_cast<double>(transform_timeout_ms_) / 1000.0));
      target_publisher_->publish(transform_target_point(message, transform));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No transform from '%s' to '%s': %s",
        message.header.frame_id.c_str(), target_frame_.c_str(), error.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  int transform_timeout_ms_{100};
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace
}  // namespace edgepick_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgepick_perception::TargetFrameTransformNode>());
  rclcpp::shutdown();
  return 0;
}
