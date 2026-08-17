#pragma once

#include <optional>
#include <string>
#include <vector>

#include "edgepick_task/grasp_state_machine.hpp"

namespace edgepick_task
{

enum class MoveItTaskPhase
{
  kPlanning,
  kExecution,
};

enum class MoveItActionOutcome
{
  kSucceeded,
  kFailed,
  kTimeout,
  kUnavailable,
};

// Stage 7 keeps MoveIt result handling explicit and testable.
//
// The ROS action client node can change later when real goal construction is
// added, but the task event vocabulary stays stable here.
std::optional<MoveItActionOutcome> parse_moveit_action_outcome(const std::string & outcome_name);
TaskEvent task_event_for_moveit_outcome(MoveItTaskPhase phase, MoveItActionOutcome outcome);
std::vector<std::string> valid_moveit_action_outcome_names();
const char * to_string(MoveItTaskPhase phase);
const char * to_string(MoveItActionOutcome outcome);

}  // namespace edgepick_task
