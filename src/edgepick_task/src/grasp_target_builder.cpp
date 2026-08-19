#include "edgepick_task/grasp_target_builder.hpp"

#include <cmath>

namespace edgepick_task
{
namespace
{

bool finite_point(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool finite_orientation(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) && std::isfinite(orientation.w);
}

}  // namespace

geometry_msgs::msg::Quaternion downward_grasp_orientation()
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 1.0;
  orientation.z = 0.0;
  orientation.w = 0.0;
  return orientation;
}

std::optional<geometry_msgs::msg::Quaternion> normalized_orientation(
  const geometry_msgs::msg::Quaternion & orientation)
{
  if (!finite_orientation(orientation)) {
    return std::nullopt;
  }

  const double length = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (length <= 1e-12) {
    return std::nullopt;
  }

  geometry_msgs::msg::Quaternion normalized;
  normalized.x = orientation.x / length;
  normalized.y = orientation.y / length;
  normalized.z = orientation.z / length;
  normalized.w = orientation.w / length;
  return normalized;
}

std::optional<GraspTargetPoses> build_grasp_target_poses(
  const geometry_msgs::msg::PointStamped & target_point,
  const GraspTargetOptions & options)
{
  if (target_point.header.frame_id.empty() || !finite_point(target_point.point)) {
    return std::nullopt;
  }
  if (!std::isfinite(options.pregrasp_offset_m) || !std::isfinite(options.grasp_z_offset_m) ||
      options.pregrasp_offset_m < 0.0 || options.grasp_z_offset_m < 0.0) {
    return std::nullopt;
  }

  const auto orientation = normalized_orientation(options.end_effector_orientation);
  if (!orientation.has_value()) {
    return std::nullopt;
  }

  GraspTargetPoses poses;
  poses.grasp_pose.header = target_point.header;
  poses.grasp_pose.pose.position.x = target_point.point.x;
  poses.grasp_pose.pose.position.y = target_point.point.y;
  poses.grasp_pose.pose.position.z = target_point.point.z + options.grasp_z_offset_m;
  poses.grasp_pose.pose.orientation = *orientation;

  poses.pregrasp_pose = poses.grasp_pose;
  poses.pregrasp_pose.pose.position.z =
    poses.grasp_pose.pose.position.z + options.pregrasp_offset_m;
  return poses;
}

}  // namespace edgepick_task
