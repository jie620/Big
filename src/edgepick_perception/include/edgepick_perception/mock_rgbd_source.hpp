#pragma once

#include <cstdint>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace edgepick_perception
{

struct MockRgbdSourceOptions
{
  std::string frame_id{"camera_color_optical_frame"};
  std::uint32_t width{640U};
  std::uint32_t height{480U};
  double fx{600.0};
  double fy{600.0};
  double cx{320.0};
  double cy{240.0};
  std::uint16_t depth_mm{600U};
};

sensor_msgs::msg::CameraInfo make_mock_camera_info(
  const MockRgbdSourceOptions & options,
  const builtin_interfaces::msg::Time & stamp);

sensor_msgs::msg::Image make_mock_depth_image(
  const MockRgbdSourceOptions & options,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace edgepick_perception
