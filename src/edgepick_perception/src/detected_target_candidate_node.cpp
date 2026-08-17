#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "edgepick_interfaces/msg/target_detection.hpp"
#include "edgepick_interfaces/msg/target_detection_array.hpp"
#include "edgepick_perception/rgbd_projection.hpp"
#include "edgepick_perception/target_detection_selection.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace edgepick_perception
{
namespace
{

TargetDetectionCandidate from_message(const edgepick_interfaces::msg::TargetDetection & message)
{
  return TargetDetectionCandidate{
    message.class_id,
    message.label,
    static_cast<double>(message.score),
    static_cast<double>(message.center_u),
    static_cast<double>(message.center_v),
    static_cast<double>(message.size_u),
    static_cast<double>(message.size_v)};
}

class DetectedTargetCandidateNode final : public rclcpp::Node
{
public:
  DetectedTargetCandidateNode()
  : Node("edgepick_detected_target_candidate")
  {
    const std::string detections_topic =
      declare_parameter<std::string>("detections_topic", "/edgepick/perception/detections");
    const std::string depth_topic =
      declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    const std::string camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/camera/depth/camera_info");
    const std::string target_topic =
      declare_parameter<std::string>("target_topic", "/edgepick/perception/target_point");
    const std::string event_topic =
      declare_parameter<std::string>("event_topic", "/edgepick/task/event");

    selection_config_.min_score = declare_parameter<double>("min_detection_score", 0.50);
    selection_config_.target_label = declare_parameter<std::string>("target_label", "");
    const int target_class_id = declare_parameter<int>("target_class_id", -1);
    if (target_class_id >= 0) {
      selection_config_.target_class_id = static_cast<std::uint32_t>(target_class_id);
    }
    max_detection_age_ = std::chrono::milliseconds{
      declare_parameter<int>("max_detection_age_ms", 500)};
    depth_range_.min_m = declare_parameter<double>("min_depth_m", 0.05);
    depth_range_.max_m = declare_parameter<double>("max_depth_m", 1.20);
    publish_task_events_ = declare_parameter<bool>("publish_task_events", true);
    publish_target_lost_ = declare_parameter<bool>("publish_target_lost", true);
    publish_event_once_ = declare_parameter<bool>("publish_event_once", true);

    target_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(target_topic, 10);
    event_publisher_ = create_publisher<std_msgs::msg::String>(event_topic, 10);
    detections_subscription_ =
      create_subscription<edgepick_interfaces::msg::TargetDetectionArray>(
      detections_topic, 10,
      [this](const edgepick_interfaces::msg::TargetDetectionArray::SharedPtr message) {
        handle_detections(*message);
      });
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        handle_camera_info(*message);
      });
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr message) { handle_depth(*message); });

    RCLCPP_INFO(
      get_logger(),
      "Detected target candidate node waiting for detections='%s' depth='%s' camera_info='%s'.",
      detections_topic.c_str(), depth_topic.c_str(), camera_info_topic.c_str());
  }

private:
  void handle_detections(const edgepick_interfaces::msg::TargetDetectionArray & message)
  {
    std::vector<TargetDetectionCandidate> candidates;
    candidates.reserve(message.detections.size());
    for (const auto & detection : message.detections) {
      candidates.push_back(from_message(detection));
    }

    latest_detection_ = select_target_detection(candidates, selection_config_);
    latest_detection_stamp_ = now();
    if (!latest_detection_.has_value()) {
      publish_event_if_enabled("target_lost", target_lost_event_sent_);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No detection matched target filters; waiting for a valid candidate.");
      return;
    }

    RCLCPP_DEBUG(
      get_logger(), "Selected detection label='%s' class_id=%u score=%.3f center=(%.1f, %.1f).",
      latest_detection_->label.c_str(), latest_detection_->class_id, latest_detection_->score,
      latest_detection_->center_u, latest_detection_->center_v);
  }

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
    if (!latest_detection_.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth image received before valid detection; skipping.");
      return;
    }
    if (now() - latest_detection_stamp_ > rclcpp::Duration(max_detection_age_)) {
      latest_detection_.reset();
      publish_event_if_enabled("target_lost", target_lost_event_sent_);
      return;
    }

    const Pixel pixel = detection_center_pixel(*latest_detection_);
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
      get_logger(), "Published detected target at pixel=(%d,%d), xyz=(%.3f, %.3f, %.3f).",
      pixel.u, pixel.v, point->x, point->y, point->z);
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

  TargetDetectionSelectionConfig selection_config_;
  std::optional<TargetDetectionCandidate> latest_detection_;
  rclcpp::Time latest_detection_stamp_;
  std::chrono::milliseconds max_detection_age_{500};
  std::optional<PinholeIntrinsics> intrinsics_;
  DepthRange depth_range_;
  bool publish_task_events_{true};
  bool publish_target_lost_{true};
  bool publish_event_once_{true};
  bool target_acquired_event_sent_{false};
  bool target_lost_event_sent_{false};
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
  rclcpp::Subscription<edgepick_interfaces::msg::TargetDetectionArray>::SharedPtr
    detections_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
};

}  // namespace
}  // namespace edgepick_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgepick_perception::DetectedTargetCandidateNode>());
  rclcpp::shutdown();
  return 0;
}
