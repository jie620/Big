#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "edgepick_perception/rgbd_projection.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace edgepick_perception
{
namespace
{

sensor_msgs::msg::CameraInfo make_camera_info()
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = "depth_camera";
  info.width = 640;
  info.height = 480;
  info.k = {600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0};
  return info;
}

sensor_msgs::msg::Image make_uint16_depth_image(std::uint16_t depth_mm)
{
  sensor_msgs::msg::Image image;
  image.width = 4;
  image.height = 3;
  image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  image.step = image.width * sizeof(std::uint16_t);
  image.data.resize(image.step * image.height, 0U);
  const std::size_t offset = static_cast<std::size_t>(1) * image.step +
                             static_cast<std::size_t>(2) * sizeof(std::uint16_t);
  image.data.at(offset) = static_cast<std::uint8_t>(depth_mm & 0xFFU);
  image.data.at(offset + 1) = static_cast<std::uint8_t>((depth_mm >> 8U) & 0xFFU);
  return image;
}

sensor_msgs::msg::Image make_float_depth_image(float depth_m)
{
  sensor_msgs::msg::Image image;
  image.width = 2;
  image.height = 2;
  image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image.step = image.width * sizeof(float);
  image.data.resize(image.step * image.height, 0U);
  const std::size_t offset = sizeof(float);
  std::memcpy(image.data.data() + offset, &depth_m, sizeof(float));
  return image;
}

TEST(RgbdProjectionTest, ExtractsIntrinsicsFromCameraInfo)
{
  const PinholeIntrinsics intrinsics = intrinsics_from_camera_info(make_camera_info());

  EXPECT_TRUE(valid_intrinsics(intrinsics));
  EXPECT_EQ(intrinsics.fx, 600.0);
  EXPECT_EQ(intrinsics.fy, 600.0);
  EXPECT_EQ(intrinsics.cx, 320.0);
  EXPECT_EQ(intrinsics.cy, 240.0);
  EXPECT_EQ(intrinsics.frame_id, "depth_camera");
}

TEST(RgbdProjectionTest, ProjectsCenterPixelToOpticalAxis)
{
  const PinholeIntrinsics intrinsics = intrinsics_from_camera_info(make_camera_info());
  const auto point = project_pixel_to_3d(intrinsics, Pixel{320, 240}, 0.60);

  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->x, 0.0, 1e-9);
  EXPECT_NEAR(point->y, 0.0, 1e-9);
  EXPECT_NEAR(point->z, 0.60, 1e-9);
}

TEST(RgbdProjectionTest, ProjectsOffsetPixelUsingPinholeModel)
{
  const PinholeIntrinsics intrinsics = intrinsics_from_camera_info(make_camera_info());
  const auto point = project_pixel_to_3d(intrinsics, Pixel{380, 180}, 1.00);

  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->x, 0.10, 1e-9);
  EXPECT_NEAR(point->y, -0.10, 1e-9);
  EXPECT_NEAR(point->z, 1.00, 1e-9);
}

TEST(RgbdProjectionTest, ReadsUint16MillimeterDepth)
{
  const sensor_msgs::msg::Image image = make_uint16_depth_image(650U);
  const auto depth_m = depth_meters_at(image, Pixel{2, 1}, DepthRange{0.05, 1.20});

  ASSERT_TRUE(depth_m.has_value());
  EXPECT_NEAR(*depth_m, 0.650, 1e-9);
}

TEST(RgbdProjectionTest, ReadsFloatMeterDepth)
{
  const sensor_msgs::msg::Image image = make_float_depth_image(0.42F);
  const auto depth_m = depth_meters_at(image, Pixel{1, 0}, DepthRange{0.05, 1.20});

  ASSERT_TRUE(depth_m.has_value());
  EXPECT_NEAR(*depth_m, 0.42, 1e-6);
}

TEST(RgbdProjectionTest, RejectsInvalidDepthAndPixel)
{
  sensor_msgs::msg::Image image = make_uint16_depth_image(0U);

  EXPECT_FALSE(depth_meters_at(image, Pixel{2, 1}, DepthRange{0.05, 1.20}).has_value());
  EXPECT_FALSE(depth_meters_at(image, Pixel{9, 9}, DepthRange{0.05, 1.20}).has_value());

  image = make_uint16_depth_image(2500U);
  EXPECT_FALSE(depth_meters_at(image, Pixel{2, 1}, DepthRange{0.05, 1.20}).has_value());
}

TEST(RgbdProjectionTest, RejectsInvalidIntrinsics)
{
  PinholeIntrinsics intrinsics = intrinsics_from_camera_info(make_camera_info());
  intrinsics.fx = 0.0;

  EXPECT_FALSE(valid_intrinsics(intrinsics));
  EXPECT_FALSE(project_pixel_to_3d(intrinsics, Pixel{320, 240}, 0.50).has_value());
}

}  // namespace
}  // namespace edgepick_perception
