#include "edgepick_task/joint_feedback.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <edgepick_hardware/dofbot_servo_mapping.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

namespace edgepick_task
{
namespace
{

using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using ServoAngles = edgepick_hardware::ServoAngles;
using RosPositions = edgepick_hardware::RosPositions;

constexpr std::size_t kArmJointCount = 5U;

struct TargetPose
{
  ServoAngles servo_angles_deg{};
  std::vector<double> arm_positions_rad;
  std::vector<double> gripper_positions_rad;
};

class MoveItZeroRestoreValidationNode final : public rclcpp::Node
{
public:
  MoveItZeroRestoreValidationNode()
  : Node("edgepick_moveit_zero_restore_validation")
  {
    arm_group_name_ = declare_parameter<std::string>("arm_group_name", "arm_group");
    gripper_group_name_ = declare_parameter<std::string>("gripper_group_name", "grip_group");
    planning_time_sec_ = std::max(0.1, declare_parameter<double>("planning_time_sec", 5.0));
    planning_attempts_ =
      std::max(1, static_cast<int>(declare_parameter<int>("planning_attempts", 10)));
    execution_attempts_ =
      std::max(1, static_cast<int>(declare_parameter<int>("execution_attempts", 2)));
    state_monitor_wait_sec_ =
      std::max(0.1, declare_parameter<double>("state_monitor_wait_sec", 3.0));
    feedback_wait_sec_ =
      std::max(0.1, declare_parameter<double>("feedback_wait_sec", 8.0));
    settle_time_ms_ =
      std::max(0, static_cast<int>(declare_parameter<int>("settle_time_ms", 500)));
    joint_tolerance_rad_ =
      std::max(0.001, declare_parameter<double>("joint_tolerance_rad", 0.06));
    velocity_scaling_ =
      clamp_scaling(declare_parameter<double>("velocity_scaling_factor", 0.1));
    acceleration_scaling_ =
      clamp_scaling(declare_parameter<double>("acceleration_scaling_factor", 0.1));

    RCLCPP_INFO(
      get_logger(),
      "MoveIt zero/restore validation ready: arm=%s gripper=%s tolerance=%.4f rad",
      arm_group_name_.c_str(), gripper_group_name_.c_str(), joint_tolerance_rad_);
  }

  int run()
  {
    arm_group_ = std::make_shared<MoveGroup>(shared_from_this(), arm_group_name_);
    gripper_group_ = std::make_shared<MoveGroup>(shared_from_this(), gripper_group_name_);
    configure_group(*arm_group_);
    configure_group(*gripper_group_);

    if (!arm_group_->startStateMonitor(state_monitor_wait_sec_)) {
      RCLCPP_ERROR(get_logger(), "MoveIt arm state monitor did not become ready.");
      return 1;
    }
    if (!gripper_group_->startStateMonitor(state_monitor_wait_sec_)) {
      RCLCPP_ERROR(get_logger(), "MoveIt gripper state monitor did not become ready.");
      return 1;
    }

    const auto zero = make_target({90.0, 90.0, 90.0, 90.0, 90.0, 30.0});
    const auto restore = make_target({90.0, 165.0, 18.0, 0.0, 90.0, 30.0});

    log_target("zero", zero);
    log_target("restore", restore);

    if (!execute_target(zero, "zero")) {
      return 1;
    }
    if (!verify_target(zero, "zero")) {
      return 1;
    }

    settle();

    if (!execute_target(restore, "restore")) {
      return 1;
    }
    if (!verify_target(restore, "restore")) {
      return 1;
    }

    RCLCPP_INFO(
      get_logger(),
      "MoveIt zero/restore validation succeeded: zero [90,90,90,90,90,30] -> "
      "restore [90,165,18,0,90,30].");
    return 0;
  }

private:
  TargetPose make_target(const ServoAngles & servo_angles_deg) const
  {
    const RosPositions ros_positions =
      edgepick_hardware::servo_degrees_to_ros_positions(servo_angles_deg);

    TargetPose target;
    target.servo_angles_deg = servo_angles_deg;
    target.arm_positions_rad.assign(ros_positions.begin(), ros_positions.begin() + kArmJointCount);
    target.gripper_positions_rad.push_back(ros_positions.back());
    return target;
  }

  void configure_group(MoveGroup & group) const
  {
    group.setPlanningTime(planning_time_sec_);
    group.setNumPlanningAttempts(static_cast<unsigned int>(planning_attempts_));
    group.allowReplanning(true);
    group.setMaxVelocityScalingFactor(velocity_scaling_);
    group.setMaxAccelerationScalingFactor(acceleration_scaling_);
    group.setGoalJointTolerance(joint_tolerance_rad_);
  }

  bool execute_target(const TargetPose & target, const std::string & label)
  {
    RCLCPP_INFO(get_logger(), "Starting MoveIt %s target.", label.c_str());

    if (!plan_and_execute(*arm_group_, target.arm_positions_rad, label + " arm")) {
      return false;
    }
    if (!plan_and_execute_gripper(target.gripper_positions_rad.front(), label + " gripper")) {
      return false;
    }

    return true;
  }

  bool plan_and_execute_gripper(double target, const std::string & label)
  {
    for (int attempt = 1; attempt <= execution_attempts_; ++attempt) {
      gripper_group_->setStartStateToCurrentState();
      gripper_group_->clearPoseTargets();
      if (!gripper_group_->setJointValueTarget("grip_joint", target)) {
        RCLCPP_WARN(
          get_logger(), "MoveIt %s could not accept grip_joint=%.4f on attempt %d/%d.",
          label.c_str(), target, attempt, execution_attempts_);
        continue;
      }

      MoveGroup::Plan plan;
      const auto plan_result = gripper_group_->plan(plan);
      if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(
          get_logger(), "MoveIt %s planning failed on attempt %d/%d with code %d.",
          label.c_str(), attempt, execution_attempts_, plan_result.val);
        continue;
      }

      if (!rclcpp::ok()) {
        return false;
      }
      const auto execute_result = gripper_group_->execute(plan);
      if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(
          get_logger(), "MoveIt %s execution failed on attempt %d/%d with code %d.",
          label.c_str(), attempt, execution_attempts_, execute_result.val);
        continue;
      }

      RCLCPP_INFO(
        get_logger(), "MoveIt %s action succeeded on attempt %d/%d.",
        label.c_str(), attempt, execution_attempts_);
      return true;
    }

    RCLCPP_ERROR(
      get_logger(), "MoveIt %s failed after %d attempts.", label.c_str(), execution_attempts_);
    return false;
  }

  bool plan_and_execute(
    MoveGroup & group,
    const std::vector<double> & target,
    const std::string & label)
  {
    for (int attempt = 1; attempt <= execution_attempts_; ++attempt) {
      group.setStartStateToCurrentState();
      group.clearPoseTargets();
      if (!group.setJointValueTarget(target)) {
        RCLCPP_WARN(
          get_logger(), "MoveIt %s could not accept the joint target on attempt %d/%d.",
          label.c_str(), attempt, execution_attempts_);
        continue;
      }

      MoveGroup::Plan plan;
      const auto plan_result = group.plan(plan);
      if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(
          get_logger(), "MoveIt %s planning failed on attempt %d/%d with code %d.",
          label.c_str(), attempt, execution_attempts_, plan_result.val);
        continue;
      }

      if (!rclcpp::ok()) {
        return false;
      }
      const auto execute_result = group.execute(plan);
      if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(
          get_logger(), "MoveIt %s execution failed on attempt %d/%d with code %d.",
          label.c_str(), attempt, execution_attempts_, execute_result.val);
        continue;
      }

      RCLCPP_INFO(
        get_logger(), "MoveIt %s action succeeded on attempt %d/%d.",
        label.c_str(), attempt, execution_attempts_);
      return true;
    }

    RCLCPP_ERROR(
      get_logger(), "MoveIt %s failed after %d attempts.", label.c_str(), execution_attempts_);
    return false;
  }

  bool verify_target(const TargetPose & target, const std::string & label)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(feedback_wait_sec_);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto actual = current_target_values(0.5);
      if (actual.has_value()) {
        const auto & arm_actual = actual->first;
        const auto & gripper_actual = actual->second;
        const double arm_error = max_abs_error(target.arm_positions_rad, arm_actual);
        const double gripper_error =
          max_abs_error(target.gripper_positions_rad, gripper_actual);
        const double max_error = std::max(arm_error, gripper_error);
        if (max_error <= joint_tolerance_rad_) {
          RCLCPP_INFO(
            get_logger(),
            "MoveIt %s feedback verified: arm_error=%.4f rad gripper_error=%.4f rad.",
            label.c_str(), arm_error, gripper_error);
          return true;
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    const auto actual = current_target_values(0.5);
    const double arm_error = actual.has_value() ?
      max_abs_error(target.arm_positions_rad, actual->first) :
      std::numeric_limits<double>::infinity();
    const double gripper_error = actual.has_value() ?
      max_abs_error(target.gripper_positions_rad, actual->second) :
      std::numeric_limits<double>::infinity();

    RCLCPP_ERROR(
      get_logger(),
      "MoveIt %s feedback verification failed: arm_error=%.4f rad gripper_error=%.4f rad "
      "limit=%.4f rad.",
      label.c_str(), arm_error, gripper_error, joint_tolerance_rad_);
    return false;
  }

  std::optional<std::pair<std::vector<double>, std::vector<double>>> current_target_values(
    double timeout_sec) const
  {
    // Read both targets from one full RobotState. The gripper group contains
    // mimic joints, so asking it for a group-value vector is not reliable.
    const auto state = arm_group_->getCurrentState(timeout_sec);
    if (!state) {
      return std::nullopt;
    }

    const auto * arm_joint_model_group = state->getJointModelGroup(arm_group_->getName());
    if (arm_joint_model_group == nullptr) {
      return std::nullopt;
    }

    std::vector<double> arm_values;
    state->copyJointGroupPositions(arm_joint_model_group, arm_values);
    if (arm_values.empty()) {
      return std::nullopt;
    }

    return std::make_pair(
      std::move(arm_values),
      std::vector<double>{state->getVariablePosition("grip_joint")});
  }

  void log_target(const std::string & label, const TargetPose & target) const
  {
    RCLCPP_INFO(
      get_logger(),
      "MoveIt %s target servo degrees: [%.0f, %.0f, %.0f, %.0f, %.0f, %.0f]; "
      "ROS radians arm=%s gripper=%s",
      label.c_str(),
      target.servo_angles_deg[0],
      target.servo_angles_deg[1],
      target.servo_angles_deg[2],
      target.servo_angles_deg[3],
      target.servo_angles_deg[4],
      target.servo_angles_deg[5],
      format_values(target.arm_positions_rad).c_str(),
      format_values(target.gripper_positions_rad).c_str());
  }


  static std::string format_values(const std::vector<double> & values)
  {
    std::ostringstream output;
    output << std::fixed << std::setprecision(4) << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        output << ", ";
      }
      output << values[index];
    }
    output << "]";
    return output.str();
  }

  void settle() const
  {
    if (settle_time_ms_ > 0) {
      rclcpp::sleep_for(std::chrono::milliseconds(settle_time_ms_));
    }
  }

  static double clamp_scaling(double value)
  {
    if (!std::isfinite(value)) {
      return 0.1;
    }
    return std::clamp(value, 0.01, 1.0);
  }

  std::string arm_group_name_;
  std::string gripper_group_name_;
  double planning_time_sec_{5.0};
  int planning_attempts_{10};
  int execution_attempts_{2};
  double state_monitor_wait_sec_{3.0};
  double feedback_wait_sec_{8.0};
  int settle_time_ms_{500};
  double joint_tolerance_rad_{0.06};
  double velocity_scaling_{0.1};
  double acceleration_scaling_{0.1};
  std::shared_ptr<MoveGroup> arm_group_;
  std::shared_ptr<MoveGroup> gripper_group_;
};

}  // namespace
}  // namespace edgepick_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node =
    std::make_shared<edgepick_task::MoveItZeroRestoreValidationNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int exit_code = 1;
  try {
    exit_code = node->run();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Validation failed: %s", error.what());
  }
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
