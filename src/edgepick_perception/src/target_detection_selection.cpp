#include "edgepick_perception/target_detection_selection.hpp"

#include <cmath>

namespace edgepick_perception
{
namespace
{

bool finite_non_negative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

double detection_area(const TargetDetectionCandidate & candidate)
{
  if (!finite_non_negative(candidate.size_u) || !finite_non_negative(candidate.size_v)) {
    return 0.0;
  }
  return candidate.size_u * candidate.size_v;
}

bool better_detection(
  const TargetDetectionCandidate & candidate,
  const TargetDetectionCandidate & current_best)
{
  if (candidate.score > current_best.score) {
    return true;
  }
  if (candidate.score < current_best.score) {
    return false;
  }
  return detection_area(candidate) > detection_area(current_best);
}

}  // namespace

bool valid_detection_candidate(
  const TargetDetectionCandidate & candidate,
  const TargetDetectionSelectionConfig & config)
{
  if (!std::isfinite(candidate.score) || candidate.score < config.min_score) {
    return false;
  }
  if (!finite_non_negative(candidate.center_u) || !finite_non_negative(candidate.center_v)) {
    return false;
  }
  if (config.require_positive_size &&
      (!std::isfinite(candidate.size_u) || !std::isfinite(candidate.size_v) ||
       candidate.size_u <= 0.0 || candidate.size_v <= 0.0)) {
    return false;
  }
  if (config.target_class_id.has_value() && candidate.class_id != *config.target_class_id) {
    return false;
  }
  if (!config.target_label.empty() && candidate.label != config.target_label) {
    return false;
  }
  return true;
}

std::optional<TargetDetectionCandidate> select_target_detection(
  const std::vector<TargetDetectionCandidate> & candidates,
  const TargetDetectionSelectionConfig & config)
{
  std::optional<TargetDetectionCandidate> best;
  for (const auto & candidate : candidates) {
    if (!valid_detection_candidate(candidate, config)) {
      continue;
    }
    if (!best.has_value() || better_detection(candidate, *best)) {
      best = candidate;
    }
  }
  return best;
}

Pixel detection_center_pixel(const TargetDetectionCandidate & candidate)
{
  return Pixel{
    static_cast<int>(std::lround(candidate.center_u)),
    static_cast<int>(std::lround(candidate.center_v))};
}

}  // namespace edgepick_perception
