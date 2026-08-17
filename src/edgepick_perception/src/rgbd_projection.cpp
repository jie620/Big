#include "edgepick_perception/rgbd_projection.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "sensor_msgs/image_encodings.hpp"

namespace edgepick_perception
{
namespace
{

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool pixel_in_bounds(const sensor_msgs::msg::Image & image, Pixel pixel)
{
  return pixel.u >= 0 && pixel.v >= 0 &&
         static_cast<unsigned int>(pixel.u) < image.width &&
         static_cast<unsigned int>(pixel.v) < image.height;
}

std::optional<std::uint16_t> read_uint16_pixel(
  const sensor_msgs::msg::Image & image,
  Pixel pixel)
{
  constexpr std::size_t kBytesPerPixel = sizeof(std::uint16_t);
  const std::size_t offset =
    static_cast<std::size_t>(pixel.v) * image.step +
    static_cast<std::size_t>(pixel.u) * kBytesPerPixel;
  if (offset + kBytesPerPixel > image.data.size()) {
    return std::nullopt;
  }

  const auto low = static_cast<std::uint16_t>(image.data.at(offset));
  const auto high = static_cast<std::uint16_t>(image.data.at(offset + 1));
  if (image.is_bigendian) {
    return static_cast<std::uint16_t>((low << 8U) | high);
  }
  return static_cast<std::uint16_t>(low | (high << 8U));
}

std::optional<float> read_float32_pixel(const sensor_msgs::msg::Image & image, Pixel pixel)
{
  constexpr std::size_t kBytesPerPixel = sizeof(float);
  const std::size_t offset =
    static_cast<std::size_t>(pixel.v) * image.step +
    static_cast<std::size_t>(pixel.u) * kBytesPerPixel;
  if (offset + kBytesPerPixel > image.data.size()) {
    return std::nullopt;
  }

  float value = 0.0F;
  std::memcpy(&value, image.data.data() + offset, kBytesPerPixel);
  return value;
}

std::optional<double> apply_depth_range(double depth_m, DepthRange range)
{
  if (!std::isfinite(depth_m) || depth_m < range.min_m || depth_m > range.max_m) {
    return std::nullopt;
  }
  return depth_m;
}

}  // namespace

bool valid_intrinsics(const PinholeIntrinsics & intrinsics)
{
  return finite_positive(intrinsics.fx) && finite_positive(intrinsics.fy) &&
         intrinsics.width > 0 && intrinsics.height > 0;
}

PinholeIntrinsics intrinsics_from_camera_info(const sensor_msgs::msg::CameraInfo & message)
{
  return PinholeIntrinsics{
    message.k.at(0),
    message.k.at(4),
    message.k.at(2),
    message.k.at(5),
    static_cast<int>(message.width),
    static_cast<int>(message.height),
    message.header.frame_id};
}

Pixel default_center_pixel(int width, int height)
{
  return Pixel{width / 2, height / 2};
}

std::optional<double> depth_meters_at(
  const sensor_msgs::msg::Image & image,
  Pixel pixel,
  DepthRange range)
{
  if (!pixel_in_bounds(image, pixel)) {
    return std::nullopt;
  }

  if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      image.encoding == "16UC1") {
    const auto raw_mm = read_uint16_pixel(image, pixel);
    if (!raw_mm.has_value() || *raw_mm == 0U) {
      return std::nullopt;
    }
    return apply_depth_range(static_cast<double>(*raw_mm) / 1000.0, range);
  }

  if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
      image.encoding == "32FC1") {
    const auto raw_m = read_float32_pixel(image, pixel);
    if (!raw_m.has_value()) {
      return std::nullopt;
    }
    return apply_depth_range(static_cast<double>(*raw_m), range);
  }

  return std::nullopt;
}

std::optional<Point3D> project_pixel_to_3d(
  const PinholeIntrinsics & intrinsics,
  Pixel pixel,
  double depth_m)
{
  if (!valid_intrinsics(intrinsics) || !finite_positive(depth_m)) {
    return std::nullopt;
  }
  if (pixel.u < 0 || pixel.v < 0 || pixel.u >= intrinsics.width || pixel.v >= intrinsics.height) {
    return std::nullopt;
  }

  return Point3D{
    (static_cast<double>(pixel.u) - intrinsics.cx) * depth_m / intrinsics.fx,
    (static_cast<double>(pixel.v) - intrinsics.cy) * depth_m / intrinsics.fy,
    depth_m};
}

}  // namespace edgepick_perception
