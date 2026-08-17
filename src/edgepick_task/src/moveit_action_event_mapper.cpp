#include "edgepick_task/moveit_action_event_mapper.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <utility>

namespace edgepick_task
{
namespace
{

using OutcomeNamePair = std::pair<const char *, MoveItActionOutcome>;

constexpr std::array<OutcomeNamePair, 5> kOutcomeNames{{
  {"success", MoveItActionOutcome::kSucceeded},
  {"succeeded", MoveItActionOutcome::kSucceeded},
  {"failure", MoveItActionOutcome::kFailed},
  {"failed", MoveItActionOutcome::kFailed},
  {"timeout", MoveItActionOutcome::kTimeout},
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

std::optional<MoveItActionOutcome> parse_moveit_action_outcome(const std::string & outcome_name)
{
  const std::string key = normalized(outcome_name);
  if (key == "unavailable") {
    return MoveItActionOutcome::kUnavailable;
  }
  const auto match = std::find_if(kOutcomeNames.begin(), kOutcomeNames.end(), [&](const auto & item) {
    return key == item.first;
  });
  if (match == kOutcomeNames.end()) {
    return std::nullopt;
  }
  return match->second;
}

TaskEvent task_event_for_moveit_outcome(MoveItTaskPhase phase, MoveItActionOutcome outcome)
{
  if (outcome == MoveItActionOutcome::kTimeout) {
    return TaskEvent::kTimeout;
  }
  if (phase == MoveItTaskPhase::kPlanning) {
    return outcome == MoveItActionOutcome::kSucceeded ? TaskEvent::kPlanSucceeded
                                                       : TaskEvent::kPlanFailed;
  }
  return outcome == MoveItActionOutcome::kSucceeded ? TaskEvent::kExecutionSucceeded
                                                     : TaskEvent::kExecutionFailed;
}

std::vector<std::string> valid_moveit_action_outcome_names()
{
  return {"success", "failure", "timeout", "unavailable"};
}

const char * to_string(MoveItTaskPhase phase)
{
  switch (phase) {
    case MoveItTaskPhase::kPlanning:
      return "planning";
    case MoveItTaskPhase::kExecution:
      return "execution";
  }
  return "unknown";
}

const char * to_string(MoveItActionOutcome outcome)
{
  switch (outcome) {
    case MoveItActionOutcome::kSucceeded:
      return "succeeded";
    case MoveItActionOutcome::kFailed:
      return "failed";
    case MoveItActionOutcome::kTimeout:
      return "timeout";
    case MoveItActionOutcome::kUnavailable:
      return "unavailable";
  }
  return "unknown";
}

}  // namespace edgepick_task
