#include <gtest/gtest.h>

#include <cmath>

#include "edgepick_perception/target_frame_transform.hpp"

namespace edgepick_perception
{
namespace
{

geometry_msgs::msg::PointStamped point_in_camera()
{
  geometry_msgs::msg::PointStamped point;
  point.header.stamp.sec = 42;
  point.header.frame_id = "camera_color_optical_frame";
  point.point.x = 0.10;
  point.point.y = 0.20;
  point.point.z = 0.30;
  return point;
}

geometry_msgs::msg::TransformStamped identity_transform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "camera_color_optical_frame";
  transform.transform.rotation.w = 1.0;
  return transform;
}

TEST(TargetFrameTransformTest, AppliesTranslationAndPreservesSampleStamp)
{
  auto transform = identity_transform();
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;

  const auto transformed = transform_target_point(point_in_camera(), transform);

  EXPECT_EQ(transformed.header.frame_id, "base_link");
  EXPECT_EQ(transformed.header.stamp.sec, 42);
  EXPECT_NEAR(transformed.point.x, 1.10, 1e-12);
  EXPECT_NEAR(transformed.point.y, 2.20, 1e-12);
  EXPECT_NEAR(transformed.point.z, 3.30, 1e-12);
}

TEST(TargetFrameTransformTest, AppliesRotationAroundZ)
{
  auto transform = identity_transform();
  const double angle_rad = M_PI / 2.0;
  transform.transform.rotation.z = std::sin(angle_rad / 2.0);
  transform.transform.rotation.w = std::cos(angle_rad / 2.0);

  auto point = point_in_camera();
  point.point.x = 1.0;
  point.point.y = 0.0;
  point.point.z = 0.0;

  const auto transformed = transform_target_point(point, transform);

  EXPECT_NEAR(transformed.point.x, 0.0, 1e-12);
  EXPECT_NEAR(transformed.point.y, 1.0, 1e-12);
  EXPECT_NEAR(transformed.point.z, 0.0, 1e-12);
}

TEST(TargetFrameTransformTest, TreatsZeroQuaternionAsIdentity)
{
  auto transform = identity_transform();
  transform.transform.rotation.w = 0.0;
  transform.transform.translation.x = 0.25;

  const auto transformed = transform_target_point(point_in_camera(), transform);

  EXPECT_NEAR(transformed.point.x, 0.35, 1e-12);
  EXPECT_NEAR(transformed.point.y, 0.20, 1e-12);
  EXPECT_NEAR(transformed.point.z, 0.30, 1e-12);
}

}  // namespace
}  // namespace edgepick_perception
