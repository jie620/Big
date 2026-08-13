#pragma once

#include <array>
#include <chrono>
#include <cstddef>

namespace edgepick_hardware
{

constexpr std::size_t kJointCount = 6;

struct JointCommand
{
  std::array<double, kJointCount> angles_deg{};
  std::chrono::milliseconds motion_time{1000};
};

struct JointLimits
{
  std::array<double, kJointCount> min_deg{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, kJointCount> max_deg{{180.0, 180.0, 180.0, 180.0, 270.0, 180.0}};
};

}  // namespace edgepick_hardware
