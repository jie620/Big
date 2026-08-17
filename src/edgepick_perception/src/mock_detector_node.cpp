#include <chrono>
#include <memory>
#include <string>

#include "edgepick_interfaces/msg/target_detection.hpp"
#include "edgepick_interfaces/msg/target_detection_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace edgepick_perception
{
namespace
{

class MockDetectorNode final : public rclcpp::Node
{
public:
  MockDetectorNode()
  : Node("edgepick_mock_detector")
  {
    detections_topic_ =
      declare_parameter<std::string>("detections_topic", "/edgepick/perception/detections");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_color_optical_frame");
    class_id_ = static_cast<std::uint32_t>(declare_parameter<int>("class_id", 1));
    label_ = declare_parameter<std::string>("label", "target");
    score_ = declare_parameter<double>("score", 0.90);
    center_u_ = declare_parameter<double>("center_u", 320.0);
    center_v_ = declare_parameter<double>("center_v", 240.0);
    size_u_ = declare_parameter<double>("size_u", 80.0);
    size_v_ = declare_parameter<double>("size_v", 80.0);
    const double publish_hz = declare_parameter<double>("publish_hz", 10.0);

    publisher_ =
      create_publisher<edgepick_interfaces::msg::TargetDetectionArray>(detections_topic_, 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_hz)),
      [this]() { publish_detection(); });

    RCLCPP_INFO(
      get_logger(), "Mock detector publishing label='%s' score=%.2f to '%s'.",
      label_.c_str(), score_, detections_topic_.c_str());
  }

private:
  void publish_detection()
  {
    edgepick_interfaces::msg::TargetDetectionArray message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;

    edgepick_interfaces::msg::TargetDetection detection;
    detection.class_id = class_id_;
    detection.label = label_;
    detection.score = static_cast<float>(score_);
    detection.center_u = static_cast<float>(center_u_);
    detection.center_v = static_cast<float>(center_v_);
    detection.size_u = static_cast<float>(size_u_);
    detection.size_v = static_cast<float>(size_v_);
    message.detections.push_back(detection);
    publisher_->publish(message);
  }

  std::string detections_topic_;
  std::string frame_id_;
  std::uint32_t class_id_{1U};
  std::string label_;
  double score_{0.90};
  double center_u_{320.0};
  double center_v_{240.0};
  double size_u_{80.0};
  double size_v_{80.0};
  rclcpp::Publisher<edgepick_interfaces::msg::TargetDetectionArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace edgepick_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgepick_perception::MockDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
