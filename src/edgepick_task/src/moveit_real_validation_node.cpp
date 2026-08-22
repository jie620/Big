#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace edgepick_task
{
class MoveItRealValidationNode final : public rclcpp::Node
{
public:
  MoveItRealValidationNode()
  : Node("edgepick_moveit_real_validation")
  {
    move_group_name_ = declare_parameter<std::string>("move_group_name", "arm_group");
    test_joint_index_ = std::max(0, static_cast<int>(declare_parameter<int>("test_joint_index", 0)));
    test_joint_delta_rad_ = declare_parameter<double>("test_joint_delta_rad", 0.05);
    planning_time_sec_ = std::max(0.1, declare_parameter<double>("planning_time_sec", 5.0));
    planning_attempts_ =
      std::max(1, static_cast<int>(declare_parameter<int>("planning_attempts", 10)));
    validation_attempts_ =
      std::max(1, static_cast<int>(declare_parameter<int>("validation_attempts", 3)));
    state_monitor_wait_sec_ = std::max(0.1, declare_parameter<double>("state_monitor_wait_sec", 2.0));
    settle_time_ms_ = std::max(0, static_cast<int>(declare_parameter<int>("settle_time_ms", 500)));
    home_tolerance_rad_ = std::max(0.0, declare_parameter<double>("home_tolerance_rad", 0.03));
    velocity_scaling_ = clamp_scaling(
      declare_parameter<double>("velocity_scaling_factor", 0.1));
    acceleration_scaling_ = clamp_scaling(
      declare_parameter<double>("acceleration_scaling_factor", 0.1));

    RCLCPP_INFO(
      get_logger(),
      "Stage 17 validation ready: group=%s joint_index=%d joint_delta=%.4f home_tolerance=%.4f",
      move_group_name_.c_str(), test_joint_index_, test_joint_delta_rad_, home_tolerance_rad_);
  }

  int run()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), move_group_name_);
    configure_move_group();

    if (!move_group_->startStateMonitor(state_monitor_wait_sec_)) {
      RCLCPP_ERROR(
        get_logger(), "Stage 17 failed: MoveIt state monitor did not become ready in time.");
      return 1;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(200));

    const auto home_joint_values = capture_current_joint_values();
    if (!home_joint_values.has_value()) {
      return 1;
    }
    if (test_joint_index_ >= static_cast<int>(home_joint_values->size())) {
      RCLCPP_ERROR(
        get_logger(), "Stage 17 failed: joint index %d is outside the %zu-joint arm group.",
        test_joint_index_, home_joint_values->size());
      return 1;
    }

    log_joint_values("captured home", *home_joint_values);

    std::vector<double> test_joint_values = *home_joint_values;
    test_joint_values[static_cast<std::size_t>(test_joint_index_)] += test_joint_delta_rad_;
    log_joint_values("test target", test_joint_values);

    if (!plan_and_execute(test_joint_values, "small delta target")) {
      return 1;
    }

    settle();

    if (!plan_and_execute(*home_joint_values, "return home")) {
      return 1;
    }

    settle();

    const auto returned_joint_values = capture_current_joint_values();
    if (!returned_joint_values.has_value()) {
      return 1;
    }
    log_joint_values("returned home", *returned_joint_values);

    const double home_error = max_abs_error(*home_joint_values, *returned_joint_values);
    RCLCPP_INFO(
      get_logger(), "Stage 17 return-home error: %.4f rad (limit %.4f rad)", home_error,
      home_tolerance_rad_);
    if (home_error > home_tolerance_rad_) {
      RCLCPP_ERROR(
        get_logger(),
        "Stage 17 failed: return-home error %.4f rad exceeded tolerance %.4f rad.",
        home_error, home_tolerance_rad_);
      return 1;
    }

    RCLCPP_INFO(
      get_logger(),
      "Stage 17 succeeded: MoveIt planned the test motion and returned to the captured home "
      "state.");
    return 0;
  }

private:
  void configure_move_group()
  {
    move_group_->setPlanningTime(planning_time_sec_);
    move_group_->setNumPlanningAttempts(static_cast<unsigned int>(planning_attempts_));
    move_group_->allowReplanning(true);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->setGoalJointTolerance(home_tolerance_rad_);
  }

  std::optional<std::vector<double>> capture_current_joint_values() const
  {
    const auto joint_values = move_group_->getCurrentJointValues();
    if (joint_values.empty()) {
      RCLCPP_ERROR(get_logger(), "Stage 17 failed: current joint values are not available.");
      return std::nullopt;
    }
    return joint_values;
  }

  bool plan_and_execute(const std::vector<double> & target_joint_values, const std::string & label)
  {
    for (int attempt = 1; attempt <= validation_attempts_; ++attempt) {
      move_group_->setStartStateToCurrentState();
      move_group_->setJointValueTarget(target_joint_values);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_result = move_group_->plan(plan);
      if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(
          get_logger(), "Stage 17 %s planning attempt %d/%d failed with code %d.", label.c_str(),
          attempt, validation_attempts_, plan_result.val);
        rclcpp::sleep_for(std::chrono::milliseconds(250));
        continue;
      }

      const auto execute_result = move_group_->execute(plan);
      if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(
          get_logger(), "Stage 17 %s execution attempt %d/%d failed with code %d.",
          label.c_str(), attempt, validation_attempts_, execute_result.val);
        rclcpp::sleep_for(std::chrono::milliseconds(250));
        continue;
      }

      RCLCPP_INFO(
        get_logger(), "Stage 17 %s succeeded on attempt %d/%d.", label.c_str(), attempt,
        validation_attempts_);
      return true;
    }

    RCLCPP_ERROR(
      get_logger(), "Stage 17 failed: %s did not succeed after %d attempts.", label.c_str(),
      validation_attempts_);
    return false;
  }

  void settle() const
  {
    if (settle_time_ms_ > 0) {
      rclcpp::sleep_for(std::chrono::milliseconds(settle_time_ms_));
    }
  }

  void log_joint_values(const std::string & label, const std::vector<double> & values) const
  {
    RCLCPP_INFO(get_logger(), "Stage 17 %s: %s", label.c_str(), format_joint_values(values).c_str());
  }

  std::string format_joint_values(const std::vector<double> & values) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(4) << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        out << ", ";
      }
      out << values[index];
    }
    out << "]";
    return out.str();
  }

  static double max_abs_error(
    const std::vector<double> & expected,
    const std::vector<double> & actual)
  {
    if (expected.size() != actual.size()) {
      return std::numeric_limits<double>::infinity();
    }

    double max_error = 0.0;
    for (std::size_t index = 0; index < expected.size(); ++index) {
      max_error = std::max(max_error, std::abs(expected[index] - actual[index]));
    }
    return max_error;
  }

  static double clamp_scaling(double value)
  {
    if (!std::isfinite(value)) {
      return 0.1;
    }
    return std::clamp(value, 0.01, 1.0);
  }

  std::string move_group_name_;
  int test_joint_index_{0};
  double test_joint_delta_rad_{0.05};
  double planning_time_sec_{5.0};
  int planning_attempts_{10};
  int validation_attempts_{3};
  double state_monitor_wait_sec_{2.0};
  int settle_time_ms_{500};
  double home_tolerance_rad_{0.03};
  double velocity_scaling_{0.1};
  double acceleration_scaling_{0.1};
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

}  // namespace edgepick_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<edgepick_task::MoveItRealValidationNode>();
  const int exit_code = node->run();
  rclcpp::shutdown();
  return exit_code;
}
