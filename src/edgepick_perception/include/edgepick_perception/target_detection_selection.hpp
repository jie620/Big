#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "edgepick_perception/rgbd_projection.hpp"

namespace edgepick_perception
{

struct TargetDetectionCandidate
{
  std::uint32_t class_id{0U};
  std::string label;
  double score{0.0};
  double center_u{0.0};
  double center_v{0.0};
  double size_u{0.0};
  double size_v{0.0};
};

struct TargetDetectionSelectionConfig
{
  double min_score{0.50};
  std::optional<std::uint32_t> target_class_id;
  std::string target_label;
  bool require_positive_size{true};
};

// Stage 9 detector boundary: pick one 2D detection before RGB-D projection.
//
// The TensorRT/YOLO node is deliberately kept outside this pure selection
// logic. That lets us test target filtering, tie-breaking, and pixel selection
// without a camera, model file, CUDA context, or ROS executor.
bool valid_detection_candidate(
  const TargetDetectionCandidate & candidate,
  const TargetDetectionSelectionConfig & config);
std::optional<TargetDetectionCandidate> select_target_detection(
  const std::vector<TargetDetectionCandidate> & candidates,
  const TargetDetectionSelectionConfig & config);
Pixel detection_center_pixel(const TargetDetectionCandidate & candidate);

}  // namespace edgepick_perception
