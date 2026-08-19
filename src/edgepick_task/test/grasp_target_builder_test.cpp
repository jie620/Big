#include <gtest/gtest.h>

#include <cmath>

#include "edgepick_task/grasp_target_builder.hpp"

namespace edgepick_task
{
namespace
{

geometry_msgs::msg::PointStamped target_point()
{
  geometry_msgs::msg::PointStamped point;
  point.header.stamp.sec = 42;
  point.header.frame_id = "base_link";
  point.point.x = 0.30;
  point.point.y = -0.10;
  point.point.z = 0.12;
  return point;
}

GraspTargetOptions default_options()
{
  GraspTargetOptions options;
  options.pregrasp_offset_m = 0.08;
  options.grasp_z_offset_m = 0.02;
  options.end_effector_orientation = downward_grasp_orientation();
  return options;
}

TEST(GraspTargetBuilderTest, BuildsGraspAndPregraspPosesFromBasePoint)
{
  const auto poses = build_grasp_target_poses(target_point(), default_options());

  ASSERT_TRUE(poses.has_value());
  EXPECT_EQ(poses->grasp_pose.header.frame_id, "base_link");
  EXPECT_EQ(poses->grasp_pose.header.stamp.sec, 42);
  EXPECT_NEAR(poses->grasp_pose.pose.position.x, 0.30, 1e-12);
  EXPECT_NEAR(poses->grasp_pose.pose.position.y, -0.10, 1e-12);
  EXPECT_NEAR(poses->grasp_pose.pose.position.z, 0.14, 1e-12);
  EXPECT_NEAR(poses->pregrasp_pose.pose.position.z, 0.22, 1e-12);
}

TEST(GraspTargetBuilderTest, UsesNormalizedEndEffectorOrientationForBothTargets)
{
  auto options = default_options();
  options.end_effector_orientation.x = 0.0;
  options.end_effector_orientation.y = 2.0;
  options.end_effector_orientation.z = 0.0;
  options.end_effector_orientation.w = 0.0;

  const auto poses = build_grasp_target_poses(target_point(), options);

  ASSERT_TRUE(poses.has_value());
  EXPECT_NEAR(poses->grasp_pose.pose.orientation.y, 1.0, 1e-12);
  EXPECT_NEAR(poses->pregrasp_pose.pose.orientation.y, 1.0, 1e-12);
}

TEST(GraspTargetBuilderTest, RejectsInvalidTargetsAndOffsets)
{
  auto invalid_point = target_point();
  invalid_point.point.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(build_grasp_target_poses(invalid_point, default_options()).has_value());

  invalid_point = target_point();
  invalid_point.header.frame_id.clear();
  EXPECT_FALSE(build_grasp_target_poses(invalid_point, default_options()).has_value());

  auto invalid_options = default_options();
  invalid_options.pregrasp_offset_m = -0.01;
  EXPECT_FALSE(build_grasp_target_poses(target_point(), invalid_options).has_value());
}

TEST(GraspTargetBuilderTest, RejectsZeroLengthOrientation)
{
  auto options = default_options();
  options.end_effector_orientation.x = 0.0;
  options.end_effector_orientation.y = 0.0;
  options.end_effector_orientation.z = 0.0;
  options.end_effector_orientation.w = 0.0;

  EXPECT_FALSE(build_grasp_target_poses(target_point(), options).has_value());
}

}  // namespace
}  // namespace edgepick_task
