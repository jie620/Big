#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace edgepick_task
{
inline double max_abs_error(const std::vector<double> & expected, const std::vector<double> & actual)
{
  if (expected.empty() || expected.size() != actual.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double result = 0.0;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (!std::isfinite(expected[i]) || !std::isfinite(actual[i])) {
      return std::numeric_limits<double>::infinity();
    }
    result = std::max(result, std::abs(expected[i] - actual[i]));
  }
  return result;
}
}  // namespace edgepick_task
