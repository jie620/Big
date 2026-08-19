#include "edgepick_task/mock_task_script.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace edgepick_task
{
namespace
{

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

std::vector<MockTaskStep> success_steps()
{
  return {
    {TaskState::kIdle, TaskEvent::kStartRequested, "mock_operator"},
    {TaskState::kPerceiving, TaskEvent::kTargetAcquired, "mock_perception"},
    {TaskState::kPlanning, TaskEvent::kPlanSucceeded, "mock_planner"},
    {TaskState::kExecuting, TaskEvent::kExecutionSucceeded, "mock_executor"},
    {TaskState::kVerifying, TaskEvent::kVerificationSucceeded, "mock_verifier"},
  };
}

std::vector<MockTaskStep> perception_recovery_steps()
{
  return {
    {TaskState::kIdle, TaskEvent::kStartRequested, "mock_operator"},
    {TaskState::kPerceiving, TaskEvent::kTargetLost, "mock_perception"},
    {TaskState::kRecovering, TaskEvent::kRecoverySucceeded, "mock_recovery"},
    {TaskState::kPerceiving, TaskEvent::kTargetAcquired, "mock_perception"},
    {TaskState::kPlanning, TaskEvent::kPlanSucceeded, "mock_planner"},
    {TaskState::kExecuting, TaskEvent::kExecutionSucceeded, "mock_executor"},
    {TaskState::kVerifying, TaskEvent::kVerificationSucceeded, "mock_verifier"},
  };
}

std::vector<MockTaskStep> planning_recovery_steps()
{
  return {
    {TaskState::kIdle, TaskEvent::kStartRequested, "mock_operator"},
    {TaskState::kPerceiving, TaskEvent::kTargetAcquired, "mock_perception"},
    {TaskState::kPlanning, TaskEvent::kPlanFailed, "mock_planner"},
    {TaskState::kRecovering, TaskEvent::kRecoverySucceeded, "mock_recovery"},
    {TaskState::kPerceiving, TaskEvent::kTargetAcquired, "mock_perception"},
    {TaskState::kPlanning, TaskEvent::kPlanSucceeded, "mock_planner"},
    {TaskState::kExecuting, TaskEvent::kExecutionSucceeded, "mock_executor"},
    {TaskState::kVerifying, TaskEvent::kVerificationSucceeded, "mock_verifier"},
  };
}

std::vector<MockTaskStep> execution_recovery_steps()
{
  return {
    {TaskState::kIdle, TaskEvent::kStartRequested, "mock_operator"},
    {TaskState::kPerceiving, TaskEvent::kTargetAcquired, "mock_perception"},
    {TaskState::kPlanning, TaskEvent::kPlanSucceeded, "mock_planner"},
    {TaskState::kExecuting, TaskEvent::kExecutionFailed, "mock_executor"},
    {TaskState::kRecovering, TaskEvent::kRecoverySucceeded, "mock_recovery"},
    {TaskState::kPerceiving, TaskEvent::kTargetAcquired, "mock_perception"},
    {TaskState::kPlanning, TaskEvent::kPlanSucceeded, "mock_planner"},
    {TaskState::kExecuting, TaskEvent::kExecutionSucceeded, "mock_executor"},
    {TaskState::kVerifying, TaskEvent::kVerificationSucceeded, "mock_verifier"},
  };
}

std::vector<MockTaskStep> verification_recovery_steps()
{
  return {
    {TaskState::kIdle, TaskEvent::kStartRequested, "mock_operator"},
    {TaskState::kPerceiving, TaskEvent::kTargetAcquired, "mock_perception"},
    {TaskState::kPlanning, TaskEvent::kPlanSucceeded, "mock_planner"},
    {TaskState::kExecuting, TaskEvent::kExecutionSucceeded, "mock_executor"},
    {TaskState::kVerifying, TaskEvent::kVerificationFailed, "mock_verifier"},
    {TaskState::kRecovering, TaskEvent::kRecoverySucceeded, "mock_recovery"},
    {TaskState::kPerceiving, TaskEvent::kTargetAcquired, "mock_perception"},
    {TaskState::kPlanning, TaskEvent::kPlanSucceeded, "mock_planner"},
    {TaskState::kExecuting, TaskEvent::kExecutionSucceeded, "mock_executor"},
    {TaskState::kVerifying, TaskEvent::kVerificationSucceeded, "mock_verifier"},
  };
}

std::vector<MockTaskStep> moveit_adapter_success_steps()
{
  return {
    {TaskState::kIdle, TaskEvent::kStartRequested, "mock_operator"},
    {TaskState::kPerceiving, TaskEvent::kTargetAcquired, "mock_perception"},
    {TaskState::kVerifying, TaskEvent::kVerificationSucceeded, "mock_verifier"},
  };
}

std::vector<MockTaskStep> system_rehearsal_success_steps()
{
  return {
    {TaskState::kIdle, TaskEvent::kStartRequested, "mock_operator"},
    {TaskState::kVerifying, TaskEvent::kVerificationSucceeded, "mock_verifier"},
  };
}

}  // namespace

MockTaskScript::MockTaskScript(std::string name, std::vector<MockTaskStep> steps)
: name_(std::move(name)), steps_(std::move(steps))
{
}

const std::string & MockTaskScript::name() const
{
  return name_;
}

const MockTaskStep * MockTaskScript::next_step() const
{
  if (complete()) {
    return nullptr;
  }
  return &steps_.at(cursor_);
}

std::optional<TaskEvent> MockTaskScript::next_event_for_state(TaskState state)
{
  const MockTaskStep * step = next_step();
  if (step == nullptr || step->expected_state != state) {
    return std::nullopt;
  }
  const TaskEvent event = step->event;
  ++cursor_;
  return event;
}

void MockTaskScript::reset()
{
  cursor_ = 0;
}

bool MockTaskScript::complete() const
{
  return cursor_ >= steps_.size();
}

std::size_t MockTaskScript::cursor() const
{
  return cursor_;
}

std::size_t MockTaskScript::step_count() const
{
  return steps_.size();
}

std::vector<std::string> valid_mock_task_scenarios()
{
  return {
    "success",
    "perception_recovery",
    "planning_recovery",
    "execution_recovery",
    "verification_recovery",
    "moveit_success",
    "system_rehearsal_success",
  };
}

MockTaskScript make_mock_task_script(const std::string & scenario_name)
{
  const std::string key = normalized(scenario_name);
  if (key == "success") {
    return MockTaskScript{"success", success_steps()};
  }
  if (key == "perception_recovery") {
    return MockTaskScript{"perception_recovery", perception_recovery_steps()};
  }
  if (key == "planning_recovery") {
    return MockTaskScript{"planning_recovery", planning_recovery_steps()};
  }
  if (key == "execution_recovery") {
    return MockTaskScript{"execution_recovery", execution_recovery_steps()};
  }
  if (key == "verification_recovery") {
    return MockTaskScript{"verification_recovery", verification_recovery_steps()};
  }
  if (key == "moveit_success") {
    return MockTaskScript{"moveit_success", moveit_adapter_success_steps()};
  }
  if (key == "system_rehearsal_success") {
    return MockTaskScript{"system_rehearsal_success", system_rehearsal_success_steps()};
  }
  throw std::invalid_argument("unsupported mock task scenario: " + scenario_name);
}

}  // namespace edgepick_task
