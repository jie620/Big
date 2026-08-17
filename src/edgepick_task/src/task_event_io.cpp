#include "edgepick_task/task_event_io.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <sstream>
#include <utility>

namespace edgepick_task
{
namespace
{

using EventNamePair = std::pair<const char *, TaskEvent>;
using StateNamePair = std::pair<const char *, TaskState>;

constexpr std::array<EventNamePair, 14> kEventNames{{
  {"start_requested", TaskEvent::kStartRequested},
  {"target_acquired", TaskEvent::kTargetAcquired},
  {"target_lost", TaskEvent::kTargetLost},
  {"plan_succeeded", TaskEvent::kPlanSucceeded},
  {"plan_failed", TaskEvent::kPlanFailed},
  {"execution_succeeded", TaskEvent::kExecutionSucceeded},
  {"execution_failed", TaskEvent::kExecutionFailed},
  {"verification_succeeded", TaskEvent::kVerificationSucceeded},
  {"verification_failed", TaskEvent::kVerificationFailed},
  {"recovery_succeeded", TaskEvent::kRecoverySucceeded},
  {"recovery_failed", TaskEvent::kRecoveryFailed},
  {"timeout", TaskEvent::kTimeout},
  {"cancel_requested", TaskEvent::kCancelRequested},
  {"reset", TaskEvent::kReset},
}};

constexpr std::array<StateNamePair, 9> kStateNames{{
  {"idle", TaskState::kIdle},
  {"perceiving", TaskState::kPerceiving},
  {"planning", TaskState::kPlanning},
  {"executing", TaskState::kExecuting},
  {"verifying", TaskState::kVerifying},
  {"recovering", TaskState::kRecovering},
  {"succeeded", TaskState::kSucceeded},
  {"failed", TaskState::kFailed},
  {"canceled", TaskState::kCanceled},
}};

std::string normalized(std::string value)
{
  value.erase(
    value.begin(),
    std::find_if(value.begin(), value.end(), [](unsigned char character) {
      return !std::isspace(character);
    }));
  value.erase(
    std::find_if(value.rbegin(), value.rend(), [](unsigned char character) {
      return !std::isspace(character);
    }).base(),
    value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

}  // namespace

std::optional<TaskEvent> parse_task_event(const std::string & event_name)
{
  const std::string key = normalized(event_name);
  const auto match = std::find_if(kEventNames.begin(), kEventNames.end(), [&](const auto & item) {
    return key == item.first;
  });
  if (match == kEventNames.end()) {
    return std::nullopt;
  }
  return match->second;
}

std::optional<TaskState> parse_task_state(const std::string & state_name)
{
  const std::string key = normalized(state_name);
  const auto match = std::find_if(kStateNames.begin(), kStateNames.end(), [&](const auto & item) {
    return key == item.first;
  });
  if (match == kStateNames.end()) {
    return std::nullopt;
  }
  return match->second;
}

std::vector<std::string> valid_task_event_names()
{
  std::vector<std::string> names;
  names.reserve(kEventNames.size());
  for (const auto & item : kEventNames) {
    names.emplace_back(item.first);
  }
  return names;
}

std::string task_status_line(const TaskStatusSnapshot & snapshot)
{
  std::ostringstream stream;
  stream << "state=" << to_string(snapshot.state)
         << " failure=" << to_string(snapshot.failure)
         << " recovery_attempts=" << snapshot.recovery_attempts
         << " terminal=" << (snapshot.terminal ? "true" : "false");
  return stream.str();
}

TaskStatusSnapshot snapshot_from_machine(const GraspStateMachine & machine)
{
  return TaskStatusSnapshot{
    machine.state(), machine.last_failure(), machine.recovery_attempts(), machine.is_terminal()};
}

}  // namespace edgepick_task
