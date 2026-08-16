#pragma once

#include <cstddef>

namespace edgepick_task
{

enum class TaskState
{
  kIdle,
  kPerceiving,
  kPlanning,
  kExecuting,
  kVerifying,
  kRecovering,
  kSucceeded,
  kFailed,
  kCanceled,
};

enum class TaskEvent
{
  kStartRequested,
  kTargetAcquired,
  kTargetLost,
  kPlanSucceeded,
  kPlanFailed,
  kExecutionSucceeded,
  kExecutionFailed,
  kVerificationSucceeded,
  kVerificationFailed,
  kRecoverySucceeded,
  kRecoveryFailed,
  kTimeout,
  kCancelRequested,
  kReset,
};

enum class TransitionStatus
{
  kAccepted,
  kRejected,
};

enum class FailureCode
{
  kNone,
  kPerceptionFailed,
  kPlanningFailed,
  kExecutionFailed,
  kVerificationFailed,
  kRecoveryFailed,
  kTimeout,
  kCanceled,
  kInvalidTransition,
};

struct TaskConfig
{
  std::size_t max_recovery_attempts{2};
};

struct TransitionResult
{
  TransitionStatus status{TransitionStatus::kRejected};
  TaskState previous_state{TaskState::kIdle};
  TaskState current_state{TaskState::kIdle};
  FailureCode failure{FailureCode::kNone};
  std::size_t recovery_attempts{0};
};

// Pure task-state core for the grasping pipeline.
//
// The class deliberately has no ROS dependencies yet. Stage 4 locks down the
// behavior we want from perception, planning, execution, verification, timeout,
// and recovery events before a ROS node starts calling MoveIt actions.
class GraspStateMachine
{
public:
  explicit GraspStateMachine(TaskConfig config = {});

  TransitionResult handle(TaskEvent event);
  void reset();

  TaskState state() const;
  FailureCode last_failure() const;
  std::size_t recovery_attempts() const;
  bool is_terminal() const;

private:
  TransitionResult accept(TaskState next_state, FailureCode failure = FailureCode::kNone);
  TransitionResult reject(FailureCode failure = FailureCode::kInvalidTransition) const;
  TransitionResult recover_or_fail(FailureCode failure);
  bool can_recover() const;

  TaskConfig config_;
  TaskState state_{TaskState::kIdle};
  FailureCode last_failure_{FailureCode::kNone};
  std::size_t recovery_attempts_{0};
};

const char * to_string(TaskState state);
const char * to_string(TaskEvent event);
const char * to_string(TransitionStatus status);
const char * to_string(FailureCode failure);

}  // namespace edgepick_task
