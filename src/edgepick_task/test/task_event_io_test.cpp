#include <gtest/gtest.h>
#include "edgepick_task/joint_feedback.hpp"

#include "edgepick_task/task_event_io.hpp"

namespace edgepick_task
{
namespace
{

TEST(JointFeedbackTest, InvalidFeedbackCannotVerifyMotion)
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(max_abs_error({0.0, 1.0}, {0.1, 0.8}), 0.2);
  EXPECT_TRUE(std::isinf(max_abs_error({}, {})));
  EXPECT_TRUE(std::isinf(max_abs_error({0.0}, {})));
  EXPECT_TRUE(std::isinf(max_abs_error({0.0}, {nan})));
  EXPECT_TRUE(std::isinf(max_abs_error({nan}, {0.0})));
}

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

TEST(TaskEventIoTest, ParsesKnownStateNames)
{
  EXPECT_EQ(parse_task_state("idle"), TaskState::kIdle);
  EXPECT_EQ(parse_task_state("  RECOVERING "), TaskState::kRecovering);
  EXPECT_EQ(parse_task_state("succeeded"), TaskState::kSucceeded);
}

TEST(TaskEventIoTest, RejectsUnknownStateNames)
{
  EXPECT_FALSE(parse_task_state("moving_arm").has_value());
  EXPECT_FALSE(parse_task_state("").has_value());
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
