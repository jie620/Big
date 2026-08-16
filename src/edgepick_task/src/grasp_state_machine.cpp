#include "edgepick_task/grasp_state_machine.hpp"

namespace edgepick_task
{

GraspStateMachine::GraspStateMachine(TaskConfig config)
: config_(config)
{
}

TransitionResult GraspStateMachine::handle(TaskEvent event)
{
  if (event == TaskEvent::kCancelRequested && !is_terminal()) {
    return accept(TaskState::kCanceled, FailureCode::kCanceled);
  }
  if (event == TaskEvent::kReset && is_terminal()) {
    reset();
    return TransitionResult{
      TransitionStatus::kAccepted, TaskState::kIdle, state_, last_failure_, recovery_attempts_};
  }

  switch (state_) {
    case TaskState::kIdle:
      if (event == TaskEvent::kStartRequested) {
        recovery_attempts_ = 0;
        return accept(TaskState::kPerceiving);
      }
      return reject();

    case TaskState::kPerceiving:
      if (event == TaskEvent::kTargetAcquired) {
        return accept(TaskState::kPlanning);
      }
      if (event == TaskEvent::kTargetLost) {
        return recover_or_fail(FailureCode::kPerceptionFailed);
      }
      if (event == TaskEvent::kTimeout) {
        return recover_or_fail(FailureCode::kTimeout);
      }
      return reject();

    case TaskState::kPlanning:
      if (event == TaskEvent::kPlanSucceeded) {
        return accept(TaskState::kExecuting);
      }
      if (event == TaskEvent::kPlanFailed) {
        return recover_or_fail(FailureCode::kPlanningFailed);
      }
      if (event == TaskEvent::kTimeout) {
        return recover_or_fail(FailureCode::kTimeout);
      }
      return reject();

    case TaskState::kExecuting:
      if (event == TaskEvent::kExecutionSucceeded) {
        return accept(TaskState::kVerifying);
      }
      if (event == TaskEvent::kExecutionFailed) {
        return recover_or_fail(FailureCode::kExecutionFailed);
      }
      if (event == TaskEvent::kTimeout) {
        return recover_or_fail(FailureCode::kTimeout);
      }
      return reject();

    case TaskState::kVerifying:
      if (event == TaskEvent::kVerificationSucceeded) {
        return accept(TaskState::kSucceeded);
      }
      if (event == TaskEvent::kVerificationFailed) {
        return recover_or_fail(FailureCode::kVerificationFailed);
      }
      if (event == TaskEvent::kTimeout) {
        return recover_or_fail(FailureCode::kTimeout);
      }
      return reject();

    case TaskState::kRecovering:
      if (event == TaskEvent::kRecoverySucceeded) {
        return accept(TaskState::kPerceiving);
      }
      if (event == TaskEvent::kRecoveryFailed) {
        return accept(TaskState::kFailed, FailureCode::kRecoveryFailed);
      }
      if (event == TaskEvent::kTimeout) {
        return accept(TaskState::kFailed, FailureCode::kTimeout);
      }
      return reject();

    case TaskState::kSucceeded:
    case TaskState::kFailed:
    case TaskState::kCanceled:
      return reject();
  }

  return reject();
}

void GraspStateMachine::reset()
{
  state_ = TaskState::kIdle;
  last_failure_ = FailureCode::kNone;
  recovery_attempts_ = 0;
}

TaskState GraspStateMachine::state() const
{
  return state_;
}

FailureCode GraspStateMachine::last_failure() const
{
  return last_failure_;
}

std::size_t GraspStateMachine::recovery_attempts() const
{
  return recovery_attempts_;
}

bool GraspStateMachine::is_terminal() const
{
  return state_ == TaskState::kSucceeded || state_ == TaskState::kFailed ||
         state_ == TaskState::kCanceled;
}

TransitionResult GraspStateMachine::accept(TaskState next_state, FailureCode failure)
{
  const TaskState previous = state_;
  state_ = next_state;
  last_failure_ = failure;
  return TransitionResult{
    TransitionStatus::kAccepted, previous, state_, last_failure_, recovery_attempts_};
}

TransitionResult GraspStateMachine::reject(FailureCode failure) const
{
  return TransitionResult{TransitionStatus::kRejected, state_, state_, failure, recovery_attempts_};
}

TransitionResult GraspStateMachine::recover_or_fail(FailureCode failure)
{
  if (!can_recover()) {
    return accept(TaskState::kFailed, failure);
  }
  ++recovery_attempts_;
  return accept(TaskState::kRecovering, failure);
}

bool GraspStateMachine::can_recover() const
{
  return recovery_attempts_ < config_.max_recovery_attempts;
}

const char * to_string(TaskState state)
{
  switch (state) {
    case TaskState::kIdle:
      return "idle";
    case TaskState::kPerceiving:
      return "perceiving";
    case TaskState::kPlanning:
      return "planning";
    case TaskState::kExecuting:
      return "executing";
    case TaskState::kVerifying:
      return "verifying";
    case TaskState::kRecovering:
      return "recovering";
    case TaskState::kSucceeded:
      return "succeeded";
    case TaskState::kFailed:
      return "failed";
    case TaskState::kCanceled:
      return "canceled";
  }
  return "unknown";
}

const char * to_string(TaskEvent event)
{
  switch (event) {
    case TaskEvent::kStartRequested:
      return "start_requested";
    case TaskEvent::kTargetAcquired:
      return "target_acquired";
    case TaskEvent::kTargetLost:
      return "target_lost";
    case TaskEvent::kPlanSucceeded:
      return "plan_succeeded";
    case TaskEvent::kPlanFailed:
      return "plan_failed";
    case TaskEvent::kExecutionSucceeded:
      return "execution_succeeded";
    case TaskEvent::kExecutionFailed:
      return "execution_failed";
    case TaskEvent::kVerificationSucceeded:
      return "verification_succeeded";
    case TaskEvent::kVerificationFailed:
      return "verification_failed";
    case TaskEvent::kRecoverySucceeded:
      return "recovery_succeeded";
    case TaskEvent::kRecoveryFailed:
      return "recovery_failed";
    case TaskEvent::kTimeout:
      return "timeout";
    case TaskEvent::kCancelRequested:
      return "cancel_requested";
    case TaskEvent::kReset:
      return "reset";
  }
  return "unknown";
}

const char * to_string(TransitionStatus status)
{
  switch (status) {
    case TransitionStatus::kAccepted:
      return "accepted";
    case TransitionStatus::kRejected:
      return "rejected";
  }
  return "unknown";
}

const char * to_string(FailureCode failure)
{
  switch (failure) {
    case FailureCode::kNone:
      return "none";
    case FailureCode::kPerceptionFailed:
      return "perception_failed";
    case FailureCode::kPlanningFailed:
      return "planning_failed";
    case FailureCode::kExecutionFailed:
      return "execution_failed";
    case FailureCode::kVerificationFailed:
      return "verification_failed";
    case FailureCode::kRecoveryFailed:
      return "recovery_failed";
    case FailureCode::kTimeout:
      return "timeout";
    case FailureCode::kCanceled:
      return "canceled";
    case FailureCode::kInvalidTransition:
      return "invalid_transition";
  }
  return "unknown";
}

}  // namespace edgepick_task
