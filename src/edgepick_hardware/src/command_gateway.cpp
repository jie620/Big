#include "edgepick_hardware/command_gateway.hpp"

#include <cmath>

namespace edgepick_hardware
{

CommandGateway::CommandGateway(CommandTransport & transport, GatewayConfig config)
: transport_(transport), config_(config)
{
}

CommandStatus CommandGateway::submit(
  const JointCommand & command,
  std::chrono::steady_clock::time_point now)
{
  ++statistics_.attempted;

  if (!has_valid_angles(command)) {
    ++statistics_.rejected;
    return CommandStatus::kInvalidJointAngle;
  }
  if (!has_valid_duration(command)) {
    ++statistics_.rejected;
    return CommandStatus::kInvalidDuration;
  }
  if (is_duplicate(command)) {
    ++statistics_.rejected;
    return CommandStatus::kDuplicateSuppressed;
  }
  if (last_accepted_at_.has_value() && now - *last_accepted_at_ < config_.min_command_interval) {
    ++statistics_.rejected;
    return CommandStatus::kRateLimited;
  }
  if (!transport_.write(command)) {
    ++statistics_.rejected;
    ++statistics_.transport_failures;
    return CommandStatus::kTransportError;
  }

  last_accepted_command_ = command;
  last_accepted_at_ = now;
  ++statistics_.accepted;
  return CommandStatus::kAccepted;
}

const GatewayStatistics & CommandGateway::statistics() const
{
  return statistics_;
}

bool CommandGateway::has_valid_angles(const JointCommand & command) const
{
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const double angle = command.angles_deg[index];
    if (!std::isfinite(angle) || angle < config_.joint_limits.min_deg[index] ||
      angle > config_.joint_limits.max_deg[index])
    {
      return false;
    }
  }
  return true;
}

bool CommandGateway::has_valid_duration(const JointCommand & command) const
{
  return command.motion_time >= config_.min_motion_time &&
         command.motion_time <= config_.max_motion_time;
}

bool CommandGateway::is_duplicate(const JointCommand & command) const
{
  if (!last_accepted_command_.has_value()) {
    return false;
  }
  if (command.motion_time != last_accepted_command_->motion_time) {
    return false;
  }
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (std::abs(command.angles_deg[index] - last_accepted_command_->angles_deg[index]) >
      config_.duplicate_epsilon_deg)
    {
      return false;
    }
  }
  return true;
}

const char * to_string(CommandStatus status)
{
  switch (status) {
    case CommandStatus::kAccepted:
      return "accepted";
    case CommandStatus::kInvalidJointAngle:
      return "invalid_joint_angle";
    case CommandStatus::kInvalidDuration:
      return "invalid_duration";
    case CommandStatus::kRateLimited:
      return "rate_limited";
    case CommandStatus::kDuplicateSuppressed:
      return "duplicate_suppressed";
    case CommandStatus::kTransportError:
      return "transport_error";
  }
  return "unknown";
}

}  // namespace edgepick_hardware
