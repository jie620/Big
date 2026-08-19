#pragma once

#include <optional>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace edgepick_task
{

struct GraspTargetOptions
{
  double pregrasp_offset_m{0.08};
  double grasp_z_offset_m{0.02};
  geometry_msgs::msg::Quaternion end_effector_orientation;
};

struct GraspTargetPoses
{
  geometry_msgs::msg::PoseStamped pregrasp_pose;
  geometry_msgs::msg::PoseStamped grasp_pose;
};

geometry_msgs::msg::Quaternion downward_grasp_orientation();

std::optional<geometry_msgs::msg::Quaternion> normalized_orientation(
  const geometry_msgs::msg::Quaternion & orientation);

std::optional<GraspTargetPoses> build_grasp_target_poses(
  const geometry_msgs::msg::PointStamped & target_point,
  const GraspTargetOptions & options);

}  // namespace edgepick_task
