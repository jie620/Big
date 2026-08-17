#include <gtest/gtest.h>

#include "edgepick_task/mock_task_script.hpp"
#include "edgepick_task/moveit_action_event_mapper.hpp"

namespace edgepick_task
{
namespace
{

TEST(MoveItActionEventMapperTest, ParsesOutcomeNames)
{
  EXPECT_EQ(parse_moveit_action_outcome("success"), MoveItActionOutcome::kSucceeded);
  EXPECT_EQ(parse_moveit_action_outcome("  FAILED "), MoveItActionOutcome::kFailed);
  EXPECT_EQ(parse_moveit_action_outcome("timeout"), MoveItActionOutcome::kTimeout);
  EXPECT_EQ(parse_moveit_action_outcome("unavailable"), MoveItActionOutcome::kUnavailable);
  EXPECT_FALSE(parse_moveit_action_outcome("maybe").has_value());
}

TEST(MoveItActionEventMapperTest, MapsPlanningOutcomesToTaskEvents)
{
  EXPECT_EQ(
    task_event_for_moveit_outcome(MoveItTaskPhase::kPlanning, MoveItActionOutcome::kSucceeded),
    TaskEvent::kPlanSucceeded);
  EXPECT_EQ(
    task_event_for_moveit_outcome(MoveItTaskPhase::kPlanning, MoveItActionOutcome::kFailed),
    TaskEvent::kPlanFailed);
  EXPECT_EQ(
    task_event_for_moveit_outcome(MoveItTaskPhase::kPlanning, MoveItActionOutcome::kUnavailable),
    TaskEvent::kPlanFailed);
  EXPECT_EQ(
    task_event_for_moveit_outcome(MoveItTaskPhase::kPlanning, MoveItActionOutcome::kTimeout),
    TaskEvent::kTimeout);
}

TEST(MoveItActionEventMapperTest, MapsExecutionOutcomesToTaskEvents)
{
  EXPECT_EQ(
    task_event_for_moveit_outcome(MoveItTaskPhase::kExecution, MoveItActionOutcome::kSucceeded),
    TaskEvent::kExecutionSucceeded);
  EXPECT_EQ(
    task_event_for_moveit_outcome(MoveItTaskPhase::kExecution, MoveItActionOutcome::kFailed),
    TaskEvent::kExecutionFailed);
  EXPECT_EQ(
    task_event_for_moveit_outcome(MoveItTaskPhase::kExecution, MoveItActionOutcome::kUnavailable),
    TaskEvent::kExecutionFailed);
  EXPECT_EQ(
    task_event_for_moveit_outcome(MoveItTaskPhase::kExecution, MoveItActionOutcome::kTimeout),
    TaskEvent::kTimeout);
}

TEST(MoveItActionEventMapperTest, MoveItSuccessScriptLeavesPlanningAndExecutionToAdapter)
{
  MockTaskScript script = make_mock_task_script("moveit_success");
  GraspStateMachine machine;

  ASSERT_EQ(script.next_event_for_state(machine.state()), TaskEvent::kStartRequested);
  machine.handle(TaskEvent::kStartRequested);
  ASSERT_EQ(script.next_event_for_state(machine.state()), TaskEvent::kTargetAcquired);
  machine.handle(TaskEvent::kTargetAcquired);

  EXPECT_EQ(machine.state(), TaskState::kPlanning);
  EXPECT_FALSE(script.next_event_for_state(machine.state()).has_value());
  machine.handle(
    task_event_for_moveit_outcome(MoveItTaskPhase::kPlanning, MoveItActionOutcome::kSucceeded));

  EXPECT_EQ(machine.state(), TaskState::kExecuting);
  EXPECT_FALSE(script.next_event_for_state(machine.state()).has_value());
  machine.handle(
    task_event_for_moveit_outcome(MoveItTaskPhase::kExecution, MoveItActionOutcome::kSucceeded));

  ASSERT_EQ(script.next_event_for_state(machine.state()), TaskEvent::kVerificationSucceeded);
  machine.handle(TaskEvent::kVerificationSucceeded);

  EXPECT_EQ(machine.state(), TaskState::kSucceeded);
  EXPECT_TRUE(script.complete());
}

TEST(MoveItActionEventMapperTest, ListsUserFacingOutcomeNames)
{
  const auto names = valid_moveit_action_outcome_names();

  EXPECT_EQ(names.front(), "success");
  EXPECT_EQ(names.back(), "unavailable");
}

}  // namespace
}  // namespace edgepick_task
