#pragma once

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace edgepick_perception
{

// Apply a TF-style transform to a target point while preserving the sample time.
//
// The input point is still a perception product; this helper only moves it into
// a new frame. Grasp pose construction, approach offsets, and MoveIt goals stay
// in later layers.
geometry_msgs::msg::PointStamped transform_target_point(
  const geometry_msgs::msg::PointStamped & point,
  const geometry_msgs::msg::TransformStamped & transform);

}  // namespace edgepick_perception
