#include <gtest/gtest.h>

#include "edgepick_perception/mock_rgbd_source.hpp"
#include "edgepick_perception/rgbd_projection.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace edgepick_perception
{
namespace
{

MockRgbdSourceOptions options()
{
  MockRgbdSourceOptions options;
  options.frame_id = "camera_color_optical_frame";
  options.width = 4U;
  options.height = 3U;
  options.fx = 500.0;
  options.fy = 510.0;
  options.cx = 2.0;
  options.cy = 1.0;
  options.depth_mm = 650U;
  return options;
}

builtin_interfaces::msg::Time stamp()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 7U;
  return stamp;
}

TEST(MockRgbdSourceTest, BuildsCameraInfoWithIntrinsics)
{
  const auto info = make_mock_camera_info(options(), stamp());

  EXPECT_EQ(info.header.frame_id, "camera_color_optical_frame");
  EXPECT_EQ(info.header.stamp.sec, 42);
  EXPECT_EQ(info.width, 4U);
  EXPECT_EQ(info.height, 3U);
  EXPECT_EQ(info.k[0], 500.0);
  EXPECT_EQ(info.k[4], 510.0);
  EXPECT_EQ(info.k[2], 2.0);
  EXPECT_EQ(info.k[5], 1.0);
}

TEST(MockRgbdSourceTest, BuildsFilledUint16DepthImage)
{
  const auto image = make_mock_depth_image(options(), stamp());

  EXPECT_EQ(image.header.frame_id, "camera_color_optical_frame");
  EXPECT_EQ(image.encoding, sensor_msgs::image_encodings::TYPE_16UC1);
  EXPECT_EQ(image.step, 4U * sizeof(std::uint16_t));
  ASSERT_EQ(image.data.size(), image.step * image.height);

  const auto depth_m = depth_meters_at(image, Pixel{2, 1}, DepthRange{0.05, 1.20});
  ASSERT_TRUE(depth_m.has_value());
  EXPECT_NEAR(*depth_m, 0.650, 1e-9);
}

}  // namespace
}  // namespace edgepick_perception
