#include <gtest/gtest.h>

#include "edgepick_task/grasp_state_machine.hpp"

namespace edgepick_task
{
namespace
{

TEST(GraspStateMachineTest, FollowsHappyPathToSuccess)
{
  GraspStateMachine machine;

  EXPECT_EQ(machine.handle(TaskEvent::kStartRequested).current_state, TaskState::kPerceiving);
  EXPECT_EQ(machine.handle(TaskEvent::kTargetAcquired).current_state, TaskState::kPlanning);
  EXPECT_EQ(machine.handle(TaskEvent::kPlanSucceeded).current_state, TaskState::kExecuting);
  EXPECT_EQ(machine.handle(TaskEvent::kExecutionSucceeded).current_state, TaskState::kVerifying);
  EXPECT_EQ(machine.handle(TaskEvent::kVerificationSucceeded).current_state, TaskState::kSucceeded);
  EXPECT_TRUE(machine.is_terminal());
  EXPECT_EQ(machine.last_failure(), FailureCode::kNone);
}

TEST(GraspStateMachineTest, RecoversFromPlanningFailureAndRetriesPerception)
{
  GraspStateMachine machine;

  machine.handle(TaskEvent::kStartRequested);
  machine.handle(TaskEvent::kTargetAcquired);
  const auto failure = machine.handle(TaskEvent::kPlanFailed);

  EXPECT_EQ(failure.current_state, TaskState::kRecovering);
  EXPECT_EQ(failure.failure, FailureCode::kPlanningFailed);
  EXPECT_EQ(failure.recovery_attempts, 1U);

  const auto recovered = machine.handle(TaskEvent::kRecoverySucceeded);
  EXPECT_EQ(recovered.current_state, TaskState::kPerceiving);
  EXPECT_EQ(machine.recovery_attempts(), 1U);
}

TEST(GraspStateMachineTest, FailsAfterRecoveryBudgetIsExhausted)
{
  GraspStateMachine machine(TaskConfig{1});

  machine.handle(TaskEvent::kStartRequested);
  machine.handle(TaskEvent::kTargetLost);
  machine.handle(TaskEvent::kRecoverySucceeded);
  const auto final_failure = machine.handle(TaskEvent::kTargetLost);

  EXPECT_EQ(final_failure.current_state, TaskState::kFailed);
  EXPECT_EQ(final_failure.failure, FailureCode::kPerceptionFailed);
  EXPECT_EQ(final_failure.recovery_attempts, 1U);
  EXPECT_TRUE(machine.is_terminal());
}

TEST(GraspStateMachineTest, TimeoutUsesRecoveryWhenAvailable)
{
  GraspStateMachine machine;

  machine.handle(TaskEvent::kStartRequested);
  machine.handle(TaskEvent::kTargetAcquired);
  machine.handle(TaskEvent::kPlanSucceeded);
  const auto timeout = machine.handle(TaskEvent::kTimeout);

  EXPECT_EQ(timeout.current_state, TaskState::kRecovering);
  EXPECT_EQ(timeout.failure, FailureCode::kTimeout);
}

TEST(GraspStateMachineTest, CancelAndResetReturnToIdle)
{
  GraspStateMachine machine;

  machine.handle(TaskEvent::kStartRequested);
  const auto canceled = machine.handle(TaskEvent::kCancelRequested);
  EXPECT_EQ(canceled.current_state, TaskState::kCanceled);
  EXPECT_TRUE(machine.is_terminal());

  const auto reset = machine.handle(TaskEvent::kReset);
  EXPECT_EQ(reset.status, TransitionStatus::kAccepted);
  EXPECT_EQ(machine.state(), TaskState::kIdle);
  EXPECT_EQ(machine.last_failure(), FailureCode::kNone);
}

TEST(GraspStateMachineTest, RejectsInvalidTransitionsWithoutChangingState)
{
  GraspStateMachine machine;

  const auto rejected = machine.handle(TaskEvent::kPlanSucceeded);

  EXPECT_EQ(rejected.status, TransitionStatus::kRejected);
  EXPECT_EQ(rejected.current_state, TaskState::kIdle);
  EXPECT_EQ(rejected.failure, FailureCode::kInvalidTransition);
  EXPECT_EQ(machine.state(), TaskState::kIdle);
}

TEST(GraspStateMachineTest, StringNamesAreStableForLogs)
{
  EXPECT_STREQ(to_string(TaskState::kRecovering), "recovering");
  EXPECT_STREQ(to_string(TaskEvent::kVerificationFailed), "verification_failed");
  EXPECT_STREQ(to_string(TransitionStatus::kRejected), "rejected");
  EXPECT_STREQ(to_string(FailureCode::kExecutionFailed), "execution_failed");
}

}  // namespace
}  // namespace edgepick_task
