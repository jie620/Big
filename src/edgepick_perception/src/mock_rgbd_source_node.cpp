#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "edgepick_perception/mock_rgbd_source.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace edgepick_perception
{
namespace
{

class MockRgbdSourceNode final : public rclcpp::Node
{
public:
  MockRgbdSourceNode()
  : Node("edgepick_mock_rgbd_source")
  {
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/camera/depth/camera_info");
    options_.frame_id = declare_parameter<std::string>("frame_id", "camera_color_optical_frame");
    options_.width = static_cast<std::uint32_t>(
      std::max(1, static_cast<int>(declare_parameter<int>("width", 640))));
    options_.height = static_cast<std::uint32_t>(
      std::max(1, static_cast<int>(declare_parameter<int>("height", 480))));
    options_.fx = declare_parameter<double>("fx", 600.0);
    options_.fy = declare_parameter<double>("fy", 600.0);
    options_.cx = declare_parameter<double>("cx", 320.0);
    options_.cy = declare_parameter<double>("cy", 240.0);
    const double depth_m = declare_parameter<double>("depth_m", 0.60);
    options_.depth_mm = static_cast<std::uint16_t>(
      std::clamp(std::lround(depth_m * 1000.0), 1L, static_cast<long>(UINT16_MAX)));
    const double publish_hz = std::max(0.1, declare_parameter<double>("publish_hz", 10.0));

    depth_publisher_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, 10);
    camera_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_hz)),
      [this]() { publish_frame(); });

    RCLCPP_INFO(
      get_logger(),
      "Mock RGB-D source publishing %ux%u depth=%u mm frame='%s' to depth='%s' camera_info='%s'.",
      options_.width, options_.height, options_.depth_mm, options_.frame_id.c_str(),
      depth_topic_.c_str(), camera_info_topic_.c_str());
  }

private:
  void publish_frame()
  {
    const auto stamp = now();
    camera_info_publisher_->publish(make_mock_camera_info(options_, stamp));
    depth_publisher_->publish(make_mock_depth_image(options_, stamp));
  }

  std::string depth_topic_;
  std::string camera_info_topic_;
  MockRgbdSourceOptions options_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace edgepick_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgepick_perception::MockRgbdSourceNode>());
  rclcpp::shutdown();
  return 0;
}
