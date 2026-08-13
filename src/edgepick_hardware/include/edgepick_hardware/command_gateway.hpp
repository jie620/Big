#pragma once

#include <chrono>
#include <cstddef>
#include <optional>

#include "edgepick_hardware/command.hpp"
#include "edgepick_hardware/transport.hpp"

namespace edgepick_hardware
{

enum class CommandStatus
{
  kAccepted,
  kInvalidJointAngle,
  kInvalidDuration,
  kRateLimited,
  kDuplicateSuppressed,
  kTransportError,
};

struct GatewayConfig
{
  JointLimits joint_limits{};
  std::chrono::milliseconds min_command_interval{20};
  std::chrono::milliseconds min_motion_time{100};
  std::chrono::milliseconds max_motion_time{30000};
  double duplicate_epsilon_deg{0.1};
};

struct GatewayStatistics
{
  std::size_t attempted{0};
  std::size_t accepted{0};
  std::size_t rejected{0};
  std::size_t transport_failures{0};
};

class CommandGateway
{
public:
  explicit CommandGateway(CommandTransport & transport, GatewayConfig config = {});

  CommandStatus submit(
    const JointCommand & command,
    std::chrono::steady_clock::time_point now);
  const GatewayStatistics & statistics() const;

private:
  bool has_valid_angles(const JointCommand & command) const;
  bool has_valid_duration(const JointCommand & command) const;
  bool is_duplicate(const JointCommand & command) const;

  CommandTransport & transport_;
  GatewayConfig config_;
  GatewayStatistics statistics_;
  std::optional<JointCommand> last_accepted_command_;
  std::optional<std::chrono::steady_clock::time_point> last_accepted_at_;
};

const char * to_string(CommandStatus status);

}  // namespace edgepick_hardware
