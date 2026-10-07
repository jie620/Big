#pragma once

#include <algorithm>
#include <cmath>

namespace edgepick_safe {

// The perception contract supplies a target center, not a full object pose.
// Use the conservative axis-aligned top-down footprints for place verification.
inline double target_footprint_coverage(
  double target_x, double target_y, double target_side,
  double zone_x, double zone_y, double zone_side)
{
  if (!std::isfinite(target_x) || !std::isfinite(target_y) ||
      !std::isfinite(target_side) || !std::isfinite(zone_x) ||
      !std::isfinite(zone_y) || !std::isfinite(zone_side) ||
      target_side <= 0.0 || zone_side <= 0.0) {
    return 0.0;
  }

  const double target_min_x = target_x - target_side / 2.0;
  const double target_max_x = target_x + target_side / 2.0;
  const double target_min_y = target_y - target_side / 2.0;
  const double target_max_y = target_y + target_side / 2.0;
  const double zone_min_x = zone_x - zone_side / 2.0;
  const double zone_max_x = zone_x + zone_side / 2.0;
  const double zone_min_y = zone_y - zone_side / 2.0;
  const double zone_max_y = zone_y + zone_side / 2.0;
  const double overlap_x = std::max(
    0.0, std::min(target_max_x, zone_max_x) - std::max(target_min_x, zone_min_x));
  const double overlap_y = std::max(
    0.0, std::min(target_max_y, zone_max_y) - std::max(target_min_y, zone_min_y));
  const double fraction = (overlap_x * overlap_y) / (target_side * target_side);
  return std::clamp(fraction, 0.0, 1.0);
}

}  // namespace edgepick_safe
