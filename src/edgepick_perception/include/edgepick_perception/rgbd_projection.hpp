#pragma once

#include <optional>
#include <string>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace edgepick_perception
{

struct PinholeIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};
  std::string frame_id;
};

struct Pixel
{
  int u{0};
  int v{0};
};

struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct DepthRange
{
  double min_m{0.05};
  double max_m{1.20};
};

// Stage 8 foundation: deterministic RGB-D math before model inference.
//
// These functions are intentionally free of OpenCV/cv_bridge so they can be
// unit tested quickly and run in a lean ROS 2 node. Object detection can later
// choose the pixel; this layer only validates depth and projects it to 3D.
bool valid_intrinsics(const PinholeIntrinsics & intrinsics);
PinholeIntrinsics intrinsics_from_camera_info(const sensor_msgs::msg::CameraInfo & message);
Pixel default_center_pixel(int width, int height);
std::optional<double> depth_meters_at(
  const sensor_msgs::msg::Image & image,
  Pixel pixel,
  DepthRange range);
std::optional<Point3D> project_pixel_to_3d(
  const PinholeIntrinsics & intrinsics,
  Pixel pixel,
  double depth_m);

}  // namespace edgepick_perception
