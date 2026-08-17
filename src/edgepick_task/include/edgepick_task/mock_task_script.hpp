#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "edgepick_task/grasp_state_machine.hpp"

namespace edgepick_task
{

struct MockTaskStep
{
  TaskState expected_state{TaskState::kIdle};
  TaskEvent event{TaskEvent::kStartRequested};
  std::string adapter_name;
};

// Small deterministic script for Stage 6 mock task closure.
//
// The script is state-gated rather than purely time-gated: it only emits the
// next event when the task node reports the expected state. This keeps the mock
// driver useful as a safety rehearsal for later perception, MoveIt, execution,
// and verification adapters.
class MockTaskScript
{
public:
  MockTaskScript(std::string name, std::vector<MockTaskStep> steps);

  const std::string & name() const;
  const MockTaskStep * next_step() const;
  std::optional<TaskEvent> next_event_for_state(TaskState state);
  void reset();
  bool complete() const;
  std::size_t cursor() const;
  std::size_t step_count() const;

private:
  std::string name_;
  std::vector<MockTaskStep> steps_;
  std::size_t cursor_{0};
};

std::vector<std::string> valid_mock_task_scenarios();
MockTaskScript make_mock_task_script(const std::string & scenario_name);

}  // namespace edgepick_task
