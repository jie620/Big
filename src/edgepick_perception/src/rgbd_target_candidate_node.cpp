#include <algorithm>
#include <memory>
#include <optional>
#include <string>

#include "edgepick_perception/rgbd_projection.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace edgepick_perception
{
namespace
{

class RgbdTargetCandidateNode final : public rclcpp::Node
{
public:
  RgbdTargetCandidateNode()
  : Node("edgepick_rgbd_target_candidate")
  {
    const std::string depth_topic =
      declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    const std::string camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/camera/depth/camera_info");
    const std::string target_topic =
      declare_parameter<std::string>("target_topic", "/edgepick/perception/target_point");
    const std::string event_topic =
      declare_parameter<std::string>("event_topic", "/edgepick/task/event");

    target_pixel_u_ = static_cast<int>(declare_parameter<int>("target_pixel_u", -1));
    target_pixel_v_ = static_cast<int>(declare_parameter<int>("target_pixel_v", -1));
    depth_range_.min_m = declare_parameter<double>("min_depth_m", 0.05);
    depth_range_.max_m = declare_parameter<double>("max_depth_m", 1.20);
    publish_task_events_ = declare_parameter<bool>("publish_task_events", true);
    publish_target_lost_ = declare_parameter<bool>("publish_target_lost", true);
    publish_event_once_ = declare_parameter<bool>("publish_event_once", true);

    target_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(target_topic, 10);
    event_publisher_ = create_publisher<std_msgs::msg::String>(event_topic, 10);
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        handle_camera_info(*message);
      });
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr message) { handle_depth(*message); });

    RCLCPP_INFO(
      get_logger(), "RGB-D target candidate node waiting for depth='%s' camera_info='%s'.",
      depth_topic.c_str(), camera_info_topic.c_str());
  }

private:
  void handle_camera_info(const sensor_msgs::msg::CameraInfo & message)
  {
    PinholeIntrinsics candidate = intrinsics_from_camera_info(message);
    if (!valid_intrinsics(candidate)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid camera intrinsics.");
      return;
    }
    intrinsics_ = candidate;
  }

  void handle_depth(const sensor_msgs::msg::Image & message)
  {
    if (!intrinsics_.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth image received before valid camera_info; skipping.");
      return;
    }

    const Pixel pixel = selected_pixel(message);
    const auto depth_m = depth_meters_at(message, pixel, depth_range_);
    if (!depth_m.has_value()) {
      publish_event_if_enabled("target_lost", target_lost_event_sent_);
      return;
    }

    const auto point = project_pixel_to_3d(*intrinsics_, pixel, *depth_m);
    if (!point.has_value()) {
      publish_event_if_enabled("target_lost", target_lost_event_sent_);
      return;
    }

    geometry_msgs::msg::PointStamped target;
    target.header = message.header;
    target.header.frame_id =
      intrinsics_->frame_id.empty() ? message.header.frame_id : intrinsics_->frame_id;
    target.point.x = point->x;
    target.point.y = point->y;
    target.point.z = point->z;
    target_publisher_->publish(target);
    publish_event_if_enabled("target_acquired", target_acquired_event_sent_);

    RCLCPP_DEBUG(
      get_logger(), "Published target point at pixel=(%d,%d), xyz=(%.3f, %.3f, %.3f).",
      pixel.u, pixel.v, point->x, point->y, point->z);
  }

  Pixel selected_pixel(const sensor_msgs::msg::Image & message) const
  {
    Pixel pixel = default_center_pixel(static_cast<int>(message.width), static_cast<int>(message.height));
    if (target_pixel_u_ >= 0) {
      pixel.u = target_pixel_u_;
    }
    if (target_pixel_v_ >= 0) {
      pixel.v = target_pixel_v_;
    }
    return pixel;
  }

  void publish_event_if_enabled(const std::string & event_name, bool & event_sent)
  {
    if (!publish_task_events_ || (publish_event_once_ && event_sent)) {
      return;
    }
    if (event_name == "target_lost" && !publish_target_lost_) {
      return;
    }

    std_msgs::msg::String event;
    event.data = event_name;
    event_publisher_->publish(event);
    event_sent = true;
  }

  std::optional<PinholeIntrinsics> intrinsics_;
  DepthRange depth_range_;
  int target_pixel_u_{-1};
  int target_pixel_v_{-1};
  bool publish_task_events_{true};
  bool publish_target_lost_{true};
  bool publish_event_once_{true};
  bool target_acquired_event_sent_{false};
  bool target_lost_event_sent_{false};
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
};

}  // namespace
}  // namespace edgepick_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgepick_perception::RgbdTargetCandidateNode>());
  rclcpp::shutdown();
  return 0;
}
