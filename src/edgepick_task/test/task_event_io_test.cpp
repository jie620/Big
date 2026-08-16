#include <gtest/gtest.h>

#include "edgepick_task/task_event_io.hpp"

namespace edgepick_task
{
namespace
{

TEST(TaskEventIoTest, ParsesKnownEventNames)
{
  EXPECT_EQ(parse_task_event("start_requested"), TaskEvent::kStartRequested);
  EXPECT_EQ(parse_task_event("  PLAN_SUCCEEDED "), TaskEvent::kPlanSucceeded);
  EXPECT_EQ(parse_task_event("verification_failed"), TaskEvent::kVerificationFailed);
}

TEST(TaskEventIoTest, RejectsUnknownEventNames)
{
  EXPECT_FALSE(parse_task_event("move_arm_now").has_value());
  EXPECT_FALSE(parse_task_event("").has_value());
}

TEST(TaskEventIoTest, ExposesAllEventNamesForCliHelp)
{
  const auto names = valid_task_event_names();

  EXPECT_EQ(names.size(), 14U);
  EXPECT_EQ(names.front(), "start_requested");
  EXPECT_EQ(names.back(), "reset");
}

TEST(TaskEventIoTest, FormatsStatusLineForLogsAndDiagnostics)
{
  const TaskStatusSnapshot snapshot{
    TaskState::kRecovering, FailureCode::kPlanningFailed, 1, false};

  EXPECT_EQ(
    task_status_line(snapshot),
    "state=recovering failure=planning_failed recovery_attempts=1 terminal=false");
}

TEST(TaskEventIoTest, CreatesSnapshotFromStateMachine)
{
  GraspStateMachine machine;
  machine.handle(TaskEvent::kStartRequested);

  const auto snapshot = snapshot_from_machine(machine);

  EXPECT_EQ(snapshot.state, TaskState::kPerceiving);
  EXPECT_EQ(snapshot.failure, FailureCode::kNone);
  EXPECT_EQ(snapshot.recovery_attempts, 0U);
  EXPECT_FALSE(snapshot.terminal);
}

}  // namespace
}  // namespace edgepick_task
