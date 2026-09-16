#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace edgepick_safe {
constexpr std::array<const char *, 6> joints{
  "Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint", "grip_joint"};
constexpr std::array<double, 6> lower{-1.570796, -1.570796, -1.570796, -1.570796, -1.570796, -1.6};
constexpr std::array<double, 6> upper{1.570796, 1.570796, 1.570796, 1.570796, 3.141592, 0.0};
inline bool fresh(double now, double stamp, double timeout) {
  return std::isfinite(stamp) && stamp > 0 && now >= stamp && now - stamp <= timeout;
}
struct Gate {
  bool armed = false, estop = true, fault = false, held = false;
  uint32_t sequence = 0;
  double estop_at = 0, joint_at = 0, target_at = 0, cloud_at = 0, scene_at = 0, policy_at = 0;
  double observation_timeout = 1.0, scene_timeout = 2.0, policy_timeout = 5.0;
  double max_step = 0.35, max_grip_step = 1.6;
  std::array<double, 6> measured{};
  std::string reason(double now) const {
    if (estop) return "emergency_stop";
    if (!fresh(now, estop_at, scene_timeout)) return "estop_timeout";
    if (fault) return "latched_fault";
    if (!armed) return "not_armed";
    if (!fresh(now, joint_at, observation_timeout)) return "joint_timeout";
    if (!fresh(now, target_at, observation_timeout)) return "target_lost";
    if (!fresh(now, cloud_at, scene_timeout)) return "depth_timeout";
    if (!fresh(now, scene_at, scene_timeout)) return "scene_timeout";
    if (!fresh(now, policy_at, policy_timeout)) return "policy_timeout";
    return "";
  }
  std::string proposal(double now, double acquired_at, double ttl, uint32_t seq,
                       const std::array<double, 6> & q) const {
    auto why = reason(now);
    if (!why.empty()) return why;
    if (!std::isfinite(ttl) || ttl <= 0 || ttl > policy_timeout ||
        !fresh(now, acquired_at, ttl)) return "expired_proposal";
    if (seq <= sequence) return "replayed_proposal";
    for (size_t i = 0; i < q.size(); ++i) {
      if (!std::isfinite(q[i]) || q[i] < lower[i] || q[i] > upper[i]) return "joint_limit";
      if (std::abs(q[i] - measured[i]) > (i == 5 ? max_grip_step : max_step)) return "action_jump";
    }
    return "";
  }
  void stop() { fault = true; armed = false; }
};
}  // namespace edgepick_safe
