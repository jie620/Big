#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "edgepick_task/grasp_state_machine.hpp"

namespace edgepick_task
{

struct TaskStatusSnapshot
{
  TaskState state{TaskState::kIdle};
  FailureCode failure{FailureCode::kNone};
  std::size_t recovery_attempts{0};
  bool terminal{false};
};

// String IO is intentionally centralized so ROS topics, logs, CLI examples, and
// tests all use the same event vocabulary.
std::optional<TaskEvent> parse_task_event(const std::string & event_name);
std::vector<std::string> valid_task_event_names();
std::string task_status_line(const TaskStatusSnapshot & snapshot);
TaskStatusSnapshot snapshot_from_machine(const GraspStateMachine & machine);

}  // namespace edgepick_task
