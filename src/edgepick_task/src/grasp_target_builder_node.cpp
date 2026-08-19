#include <memory>
#include <string>
#include <vector>

#include "edgepick_task/grasp_target_builder.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace edgepick_task
{
namespace
{

geometry_msgs::msg::Quaternion orientation_from_xyzw(const std::vector<double> & values)
{
  if (values.size() != 4U) {
    return downward_grasp_orientation();
  }

  geometry_msgs::msg::Quaternion orientation;
  orientation.x = values[0];
  orientation.y = values[1];
  orientation.z = values[2];
  orientation.w = values[3];
  return orientation;
}

class GraspTargetBuilderNode final : public rclcpp::Node
{
public:
  GraspTargetBuilderNode()
  : Node("edgepick_grasp_target_builder")
  {
    target_point_topic_ =
      declare_parameter<std::string>("target_point_topic", "/edgepick/perception/target_point_base");
    pregrasp_pose_topic_ =
      declare_parameter<std::string>("pregrasp_pose_topic", "/edgepick/task/pregrasp_pose");
    grasp_pose_topic_ =
      declare_parameter<std::string>("grasp_pose_topic", "/edgepick/task/grasp_pose");
    expected_frame_ = declare_parameter<std::string>("expected_frame", "base_link");

    options_.pregrasp_offset_m = declare_parameter<double>("pregrasp_offset_m", 0.08);
    options_.grasp_z_offset_m = declare_parameter<double>("grasp_z_offset_m", 0.02);
    options_.end_effector_orientation = orientation_from_xyzw(
      declare_parameter<std::vector<double>>("end_effector_orientation_xyzw", {0.0, 1.0, 0.0, 0.0}));

    pregrasp_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(pregrasp_pose_topic_, 10);
    grasp_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(grasp_pose_topic_, 10);
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_point_topic_, 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
        handle_target_point(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Grasp target builder watching '%s' and publishing pregrasp '%s' plus grasp '%s'.",
      target_point_topic_.c_str(), pregrasp_pose_topic_.c_str(), grasp_pose_topic_.c_str());
  }

private:
  void handle_target_point(const geometry_msgs::msg::PointStamped & message)
  {
    if (!expected_frame_.empty() && message.header.frame_id != expected_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target point frame '%s' does not match expected planning frame '%s'; skipping.",
        message.header.frame_id.c_str(), expected_frame_.c_str());
      return;
    }

    const auto poses = build_grasp_target_poses(message, options_);
    if (!poses.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target point or grasp target parameters are invalid; skipping.");
      return;
    }

    pregrasp_publisher_->publish(poses->pregrasp_pose);
    grasp_publisher_->publish(poses->grasp_pose);
  }

  std::string target_point_topic_;
  std::string pregrasp_pose_topic_;
  std::string grasp_pose_topic_;
  std::string expected_frame_;
  GraspTargetOptions options_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pregrasp_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
};

}  // namespace
}  // namespace edgepick_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edgepick_task::GraspTargetBuilderNode>());
  rclcpp::shutdown();
  return 0;
}
