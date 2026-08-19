#include "edgepick_perception/mock_rgbd_source.hpp"

#include <cstddef>

#include "sensor_msgs/image_encodings.hpp"

namespace edgepick_perception
{

sensor_msgs::msg::CameraInfo make_mock_camera_info(
  const MockRgbdSourceOptions & options,
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = options.frame_id;
  info.width = options.width;
  info.height = options.height;
  info.k = {options.fx, 0.0, options.cx, 0.0, options.fy, options.cy, 0.0, 0.0, 1.0};
  return info;
}

sensor_msgs::msg::Image make_mock_depth_image(
  const MockRgbdSourceOptions & options,
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = options.frame_id;
  image.width = options.width;
  image.height = options.height;
  image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  image.is_bigendian = 0U;
  image.step = image.width * sizeof(std::uint16_t);
  image.data.resize(static_cast<std::size_t>(image.step) * image.height, 0U);

  for (std::size_t offset = 0; offset + 1U < image.data.size(); offset += sizeof(std::uint16_t)) {
    image.data[offset] = static_cast<std::uint8_t>(options.depth_mm & 0xFFU);
    image.data[offset + 1U] = static_cast<std::uint8_t>((options.depth_mm >> 8U) & 0xFFU);
  }
  return image;
}

}  // namespace edgepick_perception
