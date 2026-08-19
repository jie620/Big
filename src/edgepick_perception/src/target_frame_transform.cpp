#include "edgepick_perception/target_frame_transform.hpp"

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"

namespace edgepick_perception
{

geometry_msgs::msg::PointStamped transform_target_point(
  const geometry_msgs::msg::PointStamped & point,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Quaternion rotation{
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w};
  if (rotation.length2() > 0.0) {
    rotation.normalize();
  } else {
    rotation.setRPY(0.0, 0.0, 0.0);
  }

  const tf2::Vector3 translation{
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z};
  const tf2::Transform frame_transform{rotation, translation};
  const tf2::Vector3 input_point{point.point.x, point.point.y, point.point.z};
  const tf2::Vector3 output_point = frame_transform * input_point;

  geometry_msgs::msg::PointStamped transformed = point;
  transformed.header.frame_id = transform.header.frame_id;
  transformed.point.x = output_point.x();
  transformed.point.y = output_point.y();
  transformed.point.z = output_point.z();
  return transformed;
}

}  // namespace edgepick_perception
