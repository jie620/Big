#include <gtest/gtest.h>

#include <stdexcept>

#include "edgepick_task/mock_task_script.hpp"

namespace edgepick_task
{
namespace
{

void expect_script_reaches_success(const std::string & scenario, std::size_t expected_recoveries)
{
  MockTaskScript script = make_mock_task_script(scenario);
  GraspStateMachine machine;

  while (!script.complete()) {
    const auto event = script.next_event_for_state(machine.state());
    ASSERT_TRUE(event.has_value()) << "scenario=" << scenario << " state="
                                   << to_string(machine.state())
                                   << " cursor=" << script.cursor();

    const TransitionResult result = machine.handle(*event);
    ASSERT_EQ(result.status, TransitionStatus::kAccepted)
      << "scenario=" << scenario << " event=" << to_string(*event);
  }

  EXPECT_EQ(machine.state(), TaskState::kSucceeded);
  EXPECT_EQ(machine.recovery_attempts(), expected_recoveries);
  EXPECT_TRUE(machine.is_terminal());
}

TEST(MockTaskScriptTest, SuccessScenarioDrivesStateMachineToSucceeded)
{
  expect_script_reaches_success("success", 0U);
}

TEST(MockTaskScriptTest, RecoveryScenariosDriveStateMachineToSucceeded)
{
  expect_script_reaches_success("perception_recovery", 1U);
  expect_script_reaches_success("planning_recovery", 1U);
  expect_script_reaches_success("execution_recovery", 1U);
  expect_script_reaches_success("verification_recovery", 1U);
}

TEST(MockTaskScriptTest, WaitsForExpectedStateBeforeAdvancing)
{
  MockTaskScript script = make_mock_task_script("success");

  EXPECT_FALSE(script.next_event_for_state(TaskState::kPlanning).has_value());
  EXPECT_EQ(script.cursor(), 0U);

  const auto event = script.next_event_for_state(TaskState::kIdle);
  ASSERT_TRUE(event.has_value());
  EXPECT_EQ(*event, TaskEvent::kStartRequested);
  EXPECT_EQ(script.cursor(), 1U);
}

TEST(MockTaskScriptTest, RejectsUnsupportedScenario)
{
  EXPECT_THROW(make_mock_task_script("hardware_now"), std::invalid_argument);
}

TEST(MockTaskScriptTest, ListsAvailableScenariosForLaunchParameters)
{
  const auto names = valid_mock_task_scenarios();

  EXPECT_EQ(names.size(), 6U);
  EXPECT_EQ(names.front(), "success");
  EXPECT_EQ(names.back(), "moveit_success");
}

}  // namespace
}  // namespace edgepick_task
