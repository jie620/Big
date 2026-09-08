#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <future>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "edgepick_hardware/command.hpp"
#include "edgepick_hardware/dofbot_i2c_transport.hpp"
#include "edgepick_task/task_event_io.hpp"

#include <dofbot_pro_interface/srv/kinemarics.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace edgepick_task
{
namespace
{

using ServoPose = std::array<double, edgepick_hardware::kJointCount>;
using Kinemarics = dofbot_pro_interface::srv::Kinemarics;

struct Matrix3
{
  std::array<std::array<double, 3>, 3> data{};
};

struct RigidTransform
{
  Matrix3 rotation{};
  std::array<double, 3> translation{};
};

struct ServoStep
{
  std::string label;
  ServoPose pose{};
  int motion_time_ms{1000};
  int settle_time_ms{250};
};

struct TrackingResult
{
  geometry_msgs::msg::PointStamped target;
  ServoPose reference_pose{};
};

class OrangeGraspExecutorNode final : public rclcpp::Node
{
public:
  OrangeGraspExecutorNode()
  : Node("edgepick_orange_grasp_executor")
  {
    event_topic_ = declare_parameter<std::string>("event_topic", "/edgepick/task/event");
    state_topic_ = declare_parameter<std::string>("state_topic", "/edgepick/task/state");
    target_point_topic_ =
      declare_parameter<std::string>("target_point_topic", "/edgepick/perception/target_point");
    startup_ready_topic_ =
      declare_parameter<std::string>("startup_ready_topic", "/edgepick/startup_pose/ready");
    require_startup_pose_ready_ =
      declare_parameter<bool>("require_startup_pose_ready", true);
    startup_pose_ready_ = !require_startup_pose_ready_;

    use_real_i2c_ = declare_parameter<bool>("use_real_i2c", true);
    i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-7");
    i2c_address_ = parse_i2c_address(declare_parameter<std::string>("i2c_address", "0x15"));
    use_kinematics_service_ = declare_parameter<bool>("use_kinematics_service", true);
    kinematics_service_name_ =
      declare_parameter<std::string>("kinematics_service_name", "dofbot_kinemarics");
    kinematics_wait_sec_ =
      std::max(0.1, declare_parameter<double>("kinematics_wait_sec", 5.0));

    state_transition_wait_sec_ =
      std::max(0.1, declare_parameter<double>("state_transition_wait_sec", 10.0));
    startup_wait_sec_ = std::max(0.1, declare_parameter<double>("startup_wait_sec", 30.0));
    target_wait_sec_ = std::max(0.1, declare_parameter<double>("target_wait_sec", 30.0));
    verification_settle_ms_ =
      std::max(0, static_cast<int>(declare_parameter<int>("verification_settle_ms", 300)));
    default_settle_ms_ =
      std::max(0, static_cast<int>(declare_parameter<int>("settle_time_ms", 250)));

    pregrasp_motion_time_ms_ = read_motion_time_ms("pregrasp_motion_time_ms", 1000);
    descend_motion_time_ms_ = read_motion_time_ms("descend_motion_time_ms", 1000);
    grip_motion_time_ms_ = read_motion_time_ms("grip_motion_time_ms", 600);
    lift_motion_time_ms_ = read_motion_time_ms("lift_motion_time_ms", 1000);
    finish_motion_time_ms_ = read_motion_time_ms("finish_motion_time_ms", 1000);

    gripper_open_angle_deg_ =
      validate_servo_angle(5U, declare_parameter<double>("gripper_open_angle_deg", 30.0));
    gripper_close_angle_deg_ =
      validate_servo_angle(5U, declare_parameter<double>("gripper_close_angle_deg", 142.0));

    apply_target_yaw_ = declare_parameter<bool>("apply_target_yaw", true);
    target_yaw_gain_ = declare_parameter<double>("target_yaw_gain", 1.0);
    target_yaw_offset_deg_ = declare_parameter<double>("target_yaw_offset_deg", 0.0);
    target_yaw_min_deg_ = declare_parameter<double>("target_yaw_min_deg", 20.0);
    target_yaw_max_deg_ = declare_parameter<double>("target_yaw_max_deg", 160.0);
    tracking_enabled_ = declare_parameter<bool>("tracking_enabled", true);
    tracking_max_updates_ = std::clamp(
      static_cast<int>(declare_parameter<int>("tracking_max_updates", 6)), 1, 100);
    tracking_centered_updates_ = std::clamp(
      static_cast<int>(declare_parameter<int>("tracking_centered_updates", 2)),
      1, tracking_max_updates_);
    tracking_center_tolerance_deg_ = std::clamp(
      declare_parameter<double>("tracking_center_tolerance_deg", 4.0), 0.1, 45.0);
    tracking_yaw_gain_ = std::clamp(
      declare_parameter<double>("tracking_yaw_gain", 0.65), 0.01, 1.0);
    tracking_motion_time_ms_ = read_motion_time_ms("tracking_motion_time_ms", 250);
    tracking_target_max_age_ms_ = std::clamp(
      static_cast<int>(declare_parameter<int>("tracking_target_max_age_ms", 500)),
      20, 30000);
    target_point_mode_ = declare_parameter<std::string>("target_point_mode", "camera_optical");
    expected_target_frame_ = declare_parameter<std::string>(
      "expected_target_frame", target_point_mode_ == "base" ? "base_link" : "camera_color_optical_frame");
    target_world_offset_x_m_ = declare_parameter<double>("target_world_offset_x_m", 0.0);
    target_world_offset_y_m_ = declare_parameter<double>("target_world_offset_y_m", 0.0);
    target_world_offset_z_m_ = declare_parameter<double>("target_world_offset_z_m", 0.0);
    ik_target_z_radius_origin_m_ =
      declare_parameter<double>("ik_target_z_radius_origin_m", 0.181);
    ik_target_z_radius_gain_ = declare_parameter<double>("ik_target_z_radius_gain", 0.15);
    ik_joint4_max_deg_ =
      validate_servo_angle(3U, declare_parameter<double>("ik_joint4_max_deg", 90.0));
    ik_wrist_angle_deg_ =
      validate_servo_angle(4U, declare_parameter<double>("ik_wrist_angle_deg", 90.0));
    use_ik_joint5_ = declare_parameter<bool>("use_ik_joint5", false);
    ik_lift_servo2_angle_deg_ =
      validate_servo_angle(1U, declare_parameter<double>("ik_lift_servo2_angle_deg", 120.0));
    min_target_depth_m_ = declare_parameter<double>("min_target_depth_m", 0.05);
    max_target_depth_m_ = declare_parameter<double>("max_target_depth_m", 1.20);
    max_abs_target_lateral_m_ = declare_parameter<double>("max_abs_target_lateral_m", 0.35);
    min_target_vertical_m_ = declare_parameter<double>("min_target_vertical_m", -0.35);
    max_target_vertical_m_ = declare_parameter<double>("max_target_vertical_m", 0.35);
    return_to_finish_pose_ = declare_parameter<bool>("return_to_finish_pose", false);

    pregrasp_servo_angles_deg_ = read_servo_pose(
      "pregrasp_servo_angle", ServoPose{{90.0, 80.0, 50.0, 50.0, 90.0, 30.0}});
    grasp_servo_angles_deg_ = read_servo_pose(
      "grasp_servo_angle", ServoPose{{90.0, 35.0, 65.0, 0.0, 90.0, 30.0}});
    lift_servo_angles_deg_ = read_servo_pose(
      "lift_servo_angle", ServoPose{{90.0, 80.0, 50.0, 50.0, 90.0, 135.0}});
    finish_servo_angles_deg_ = read_servo_pose(
      "finish_servo_angle", ServoPose{{90.0, 80.0, 50.0, 50.0, 90.0, 135.0}});
    ik_reference_servo_angles_deg_ = read_servo_pose(
      "ik_reference_servo_angle", ServoPose{{90.0, 120.0, 0.0, 0.0, 90.0, 20.0}});

    if (!std::isfinite(target_yaw_min_deg_) || !std::isfinite(target_yaw_max_deg_) ||
      target_yaw_min_deg_ < 0.0 || target_yaw_max_deg_ > 180.0 ||
      target_yaw_min_deg_ > target_yaw_max_deg_ ||
      !std::isfinite(tracking_yaw_gain_) || !std::isfinite(tracking_center_tolerance_deg_))
    {
      throw std::invalid_argument("invalid tracking/yaw limits");
    }

    event_publisher_ = create_publisher<std_msgs::msg::String>(event_topic_, 10);
    if (use_kinematics_service_) {
      kinematics_client_ = create_client<Kinemarics>(kinematics_service_name_);
    }
    state_subscription_ = create_subscription<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) { handle_state_message(*message); });
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_point_topic_, 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
        handle_target_point(*message);
      });
    startup_ready_subscription_ = create_subscription<std_msgs::msg::String>(
      startup_ready_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        if (message->data == "ready") {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            startup_pose_ready_ = true;
          }
          cv_.notify_all();
          RCLCPP_INFO(get_logger(), "Startup pose sequence is ready.");
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "Vendor-style orange grasp executor ready: target=%s direct_i2c=%s device=%s "
      "address=0x%02x ik=%s service=%s tracking=%s",
      target_point_topic_.c_str(), use_real_i2c_ ? "true" : "false", i2c_device_.c_str(),
      static_cast<unsigned int>(i2c_address_), use_kinematics_service_ ? "true" : "false",
      kinematics_service_name_.c_str(), tracking_enabled_ ? "true" : "false");
  }

  void start()
  {
    worker_ = std::thread([this]() { run(); });
  }

  ~OrangeGraspExecutorNode() override
  {
    stop_requested_ = true;
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  int exit_code() const
  {
    return exit_code_.load();
  }

private:
  void handle_state_message(const std_msgs::msg::String & message)
  {
    const auto state = parse_task_state(message.data);
    if (!state.has_value()) {
      RCLCPP_WARN(get_logger(), "Ignoring unknown task state '%s'.", message.data.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_state_ = *state;
      if (*state == TaskState::kCanceled || *state == TaskState::kFailed ||
        *state == TaskState::kRecovering)
      {
        stop_requested_ = true;
      }
    }
    cv_.notify_all();
  }

  void handle_target_point(const geometry_msgs::msg::PointStamped & message)
  {
    if (message.header.frame_id != expected_target_frame_ || !target_within_bounds(message)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid target or unexpected coordinate frame.");
      return;
    }

    const auto stamp = rclcpp::Time(message.header.stamp);
    const auto age = now() - stamp;
    if (stamp.nanoseconds() <= 0 || age.nanoseconds() < 0 ||
      age > rclcpp::Duration(std::chrono::milliseconds{tracking_target_max_age_ms_}))
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (latest_target_point_ && stamp <= rclcpp::Time(latest_target_point_->header.stamp)) {
        return;
      }
      latest_target_point_ = message;
      latest_target_received_at_ = std::chrono::steady_clock::now();
    }
    cv_.notify_all();
  }

  void run()
  {
    struct ShutdownGuard
    {
      ~ShutdownGuard()
      {
        if (rclcpp::ok()) {
          rclcpp::shutdown();
        }
      }
    } shutdown_guard;

    bool succeeded = false;
    try {
      if (!wait_for_startup_ready()) {
        publish_event(TaskEvent::kTimeout);
        return;
      }

      if (!wait_for_target_and_planning_state()) {
        publish_event(TaskEvent::kTimeout);
        return;
      }

      edgepick_hardware::DofbotI2cConfig config;
      config.enabled = use_real_i2c_;
      config.device = i2c_device_;
      config.address = i2c_address_;
      edgepick_hardware::DofbotI2cTransport transport(config);

      if (use_real_i2c_) {
        const auto feedback = transport.read_servo_angles();
        if (!feedback) {
          throw std::runtime_error("Cannot plan without current servo feedback");
        }
        ik_reference_servo_angles_deg_ = *feedback;
      }
      std::optional<TrackingResult> tracking_result;
      if (tracking_enabled_) {
        tracking_result = track_target(transport);
        if (!tracking_result.has_value()) {
          publish_event(TaskEvent::kPlanFailed);
          return;
        }
      }

      const auto target =
        tracking_result.has_value() ? std::optional{tracking_result->target} :
        capture_fresh_target_point();
      if (!target.has_value() || !target_within_bounds(*target)) {
        publish_event(TaskEvent::kPlanFailed);
        return;
      }

      const auto steps = build_grasp_steps(
        *target, tracking_result.has_value() ? tracking_result->reference_pose :
        ik_reference_servo_angles_deg_);
      if (steps.empty() || now() - rclcpp::Time(target->header.stamp) >
        rclcpp::Duration(std::chrono::milliseconds{tracking_target_max_age_ms_}))
      {
        publish_event(TaskEvent::kPlanFailed);
        return;
      }
      publish_event(TaskEvent::kPlanSucceeded);

      if (!wait_for_state(TaskState::kExecuting)) {
        publish_event(TaskEvent::kTimeout);
        return;
      }
      if (now() - rclcpp::Time(target->header.stamp) >
        rclcpp::Duration(std::chrono::milliseconds{tracking_target_max_age_ms_}))
      {
        publish_event(TaskEvent::kExecutionFailed);
        return;
      }

      if (!execute_steps(transport, steps)) {
        publish_event(TaskEvent::kExecutionFailed);
        return;
      }

      publish_event(TaskEvent::kExecutionSucceeded);
      if (!wait_for_state(TaskState::kVerifying)) {
        publish_event(TaskEvent::kTimeout);
        return;
      }

      if (!settle(verification_settle_ms_)) {
        return;
      }
      if (use_real_i2c_) {
        RCLCPP_INFO(get_logger(), "Motion sequence complete; awaiting external grasp verification.");
      } else {
        publish_event(TaskEvent::kVerificationSucceeded);
      }
      if (!wait_for_state(TaskState::kSucceeded)) {
        publish_event(TaskEvent::kTimeout);
        return;
      }
      RCLCPP_INFO(get_logger(), "Vendor-style orange grasp sequence completed successfully.");
      succeeded = true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Vendor-style orange grasp execution failed: %s", error.what());
      TaskState state;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        state = current_state_;
      }
      publish_event(state == TaskState::kExecuting ? TaskEvent::kExecutionFailed :
        TaskEvent::kPlanFailed);
    }

    exit_code_ = succeeded ? 0 : 1;
  }

  bool wait_for_startup_ready()
  {
    if (!require_startup_pose_ready_) {
      return true;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    const auto timeout = std::chrono::duration<double>(startup_wait_sec_);
    const bool ready = cv_.wait_for(lock, timeout, [this]() {
      return stop_requested_ || startup_pose_ready_;
    });
    if (!ready || stop_requested_) {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for startup pose readiness.");
      return false;
    }
    return true;
  }

  bool wait_for_target_and_planning_state()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto timeout = std::chrono::duration<double>(target_wait_sec_);
    const bool ready = cv_.wait_for(lock, timeout, [this]() {
      return stop_requested_ ||
             (startup_pose_ready_ && current_state_ == TaskState::kPlanning &&
              latest_target_point_.has_value() && target_is_fresh_locked());
    });
    if (!ready || stop_requested_) {
      RCLCPP_ERROR(
        get_logger(),
        "Timed out waiting for planning state and target point. Current state=%s "
        "target_seen=%s",
        to_string(current_state_), latest_target_point_.has_value() ? "true" : "false");
      return false;
    }
    return true;
  }

  bool wait_for_state(TaskState expected_state)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto timeout = std::chrono::duration<double>(state_transition_wait_sec_);
    const bool reached = cv_.wait_for(lock, timeout, [this, expected_state]() {
      return stop_requested_ || current_state_ == expected_state;
    });
    if (!reached || stop_requested_) {
      RCLCPP_ERROR(
        get_logger(), "Timed out waiting for task state %s; current=%s",
        to_string(expected_state), to_string(current_state_));
      return false;
    }
    return true;
  }

  std::optional<geometry_msgs::msg::PointStamped> capture_fresh_target_point()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_target_point_.has_value() || !latest_target_received_at_.has_value()) {
      return std::nullopt;
    }
    const auto age = std::chrono::steady_clock::now() - *latest_target_received_at_;
    if (age > std::chrono::milliseconds{tracking_target_max_age_ms_}) {
      return std::nullopt;
    }
    const auto age_ros = now() - rclcpp::Time(latest_target_point_->header.stamp);
    if (age_ros.nanoseconds() < 0 ||
      age_ros > rclcpp::Duration(std::chrono::milliseconds{tracking_target_max_age_ms_}))
    {
      return std::nullopt;
    }
    return latest_target_point_;
  }

  bool target_is_fresh_locked() const
  {
    return latest_target_point_.has_value() && latest_target_received_at_.has_value() &&
           std::chrono::steady_clock::now() - *latest_target_received_at_ <=
           std::chrono::milliseconds{tracking_target_max_age_ms_};
  }

  std::optional<TrackingResult> track_target(edgepick_hardware::DofbotI2cTransport & transport)
  {
    if (target_point_mode_ != "camera_optical") {
      RCLCPP_ERROR(
        get_logger(), "Orange tracking requires target_point_mode='camera_optical', got '%s'.",
        target_point_mode_.c_str());
      return std::nullopt;
    }

    ServoPose tracking_pose = ik_reference_servo_angles_deg_;
    tracking_pose[5] = gripper_open_angle_deg_;
    int centered_updates = 0;

    for (int update = 1; update <= tracking_max_updates_; ++update) {
      const auto target = capture_fresh_target_point();
      if (!target.has_value() || !target_within_bounds(*target)) {
        RCLCPP_ERROR(get_logger(), "Orange tracking lost a fresh in-range target.");
        return std::nullopt;
      }

      const double yaw_error_deg = target_lateral_yaw_error_deg(*target);
      centered_updates =
        std::abs(yaw_error_deg) <= tracking_center_tolerance_deg_ ? centered_updates + 1 : 0;
      RCLCPP_INFO(
        get_logger(),
        "Orange tracking update %d/%d: yaw_error=%.1f deg base=%.1f deg centered=%d/%d.",
        update, tracking_max_updates_, yaw_error_deg, tracking_pose[0], centered_updates,
        tracking_centered_updates_);
      if (centered_updates >= tracking_centered_updates_) {
        RCLCPP_INFO(get_logger(), "Orange tracking centered; starting grasp.");
        return TrackingResult{*target, tracking_pose};
      }
      if (centered_updates == 0) {
        tracking_pose[0] = std::clamp(
          tracking_pose[0] + tracking_yaw_gain_ * yaw_error_deg,
          target_yaw_min_deg_, target_yaw_max_deg_);
        if (!send_pose(transport, {"track orange", tracking_pose, tracking_motion_time_ms_, 0})) {
          return std::nullopt;
        }
        if (use_real_i2c_) {
          const auto feedback = transport.read_servo_angles();
          if (!feedback) {
            return std::nullopt;
          }
          tracking_pose = *feedback;
        }
      }
      // Wait for a sample captured after motion completion, not a queued old frame.
      const auto after_motion = now();
      std::unique_lock<std::mutex> lock(mutex_);
      if (!cv_.wait_for(lock, std::chrono::milliseconds{tracking_target_max_age_ms_}, [&]() {
          return stop_requested_ || (latest_target_point_ &&
            rclcpp::Time(latest_target_point_->header.stamp) > after_motion);
        }) || stop_requested_)
      {
        return std::nullopt;
      }
    }

    RCLCPP_ERROR(
      get_logger(), "Orange tracking did not center within %d updates.", tracking_max_updates_);
    return std::nullopt;
  }

  bool target_within_bounds(const geometry_msgs::msg::PointStamped & target) const
  {
    const auto & point = target.point;
    const bool finite =
      std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    if (!finite) {
      RCLCPP_ERROR(get_logger(), "Orange target point contains a non-finite coordinate.");
      return false;
    }

    const bool camera_mode = target_point_mode_ == "camera_optical";
    const bool base_mode = target_point_mode_ == "base";
    if (!camera_mode && !base_mode) {
      RCLCPP_ERROR(
        get_logger(), "target_point_mode must be 'camera_optical' or 'base', got '%s'.",
        target_point_mode_.c_str());
      return false;
    }

    const double depth = camera_mode ? point.z : point.x;
    const double lateral = camera_mode ? point.x : point.y;
    const double vertical = camera_mode ? point.y : point.z;
    const bool in_bounds =
      depth >= min_target_depth_m_ && depth <= max_target_depth_m_ &&
      std::abs(lateral) <= max_abs_target_lateral_m_ &&
      vertical >= min_target_vertical_m_ && vertical <= max_target_vertical_m_;
    if (!in_bounds) {
      RCLCPP_ERROR(
        get_logger(),
        "Orange target is outside the direct-grasp safety window: mode=%s frame=%s "
        "depth=%.3f lateral=%.3f vertical=%.3f raw=(%.3f, %.3f, %.3f)",
        target_point_mode_.c_str(), target.header.frame_id.c_str(), depth, lateral, vertical,
        point.x, point.y, point.z);
      return false;
    }
    return true;
  }

  std::vector<ServoStep> build_grasp_steps(
    const geometry_msgs::msg::PointStamped & target,
    const ServoPose & ik_reference_pose)
  {
    if (use_kinematics_service_) {
      const auto ik_pose = build_ik_grasp_pose(target, ik_reference_pose);
      if (!ik_pose.has_value()) {
        return {};
      }
      return build_ik_grasp_steps(*ik_pose);
    }

    const double yaw_deg = target_yaw_deg(target);
    RCLCPP_INFO(
      get_logger(),
      "Captured orange target: frame=%s x=%.3f y=%.3f z=%.3f -> base servo yaw %.1f deg.",
      target.header.frame_id.c_str(), target.point.x, target.point.y, target.point.z, yaw_deg);

    ServoPose pregrasp = pose_with_yaw_and_gripper(
      pregrasp_servo_angles_deg_, yaw_deg, gripper_open_angle_deg_);
    ServoPose grasp_open = pose_with_yaw_and_gripper(
      grasp_servo_angles_deg_, yaw_deg, gripper_open_angle_deg_);
    ServoPose grasp_closed = pose_with_yaw_and_gripper(
      grasp_servo_angles_deg_, yaw_deg, gripper_close_angle_deg_);
    ServoPose lift = pose_with_yaw_and_gripper(
      lift_servo_angles_deg_, yaw_deg, gripper_close_angle_deg_);
    ServoPose finish = pose_with_yaw_and_gripper(
      finish_servo_angles_deg_, yaw_deg, gripper_close_angle_deg_);

    std::vector<ServoStep> steps{
      {"pregrasp/open", pregrasp, pregrasp_motion_time_ms_, default_settle_ms_},
      {"descend", grasp_open, descend_motion_time_ms_, default_settle_ms_},
      {"close gripper", grasp_closed, grip_motion_time_ms_, default_settle_ms_},
      {"lift/hold", lift, lift_motion_time_ms_, default_settle_ms_},
    };
    if (return_to_finish_pose_) {
      steps.push_back({"finish/hold", finish, finish_motion_time_ms_, default_settle_ms_});
    }
    return steps;
  }

  std::optional<ServoPose> build_ik_grasp_pose(
    const geometry_msgs::msg::PointStamped & target,
    const ServoPose & ik_reference_pose)
  {
    const auto fk = call_fk(ik_reference_pose);
    if (!fk.has_value()) {
      return std::nullopt;
    }

    const auto world_target = target_to_vendor_world(target, *fk);
    Kinemarics::Request request;
    request.tar_x = world_target[0];
    request.tar_y = world_target[1];
    request.tar_z =
      world_target[2] +
      (std::hypot(request.tar_x, request.tar_y) - ik_target_z_radius_origin_m_) *
      ik_target_z_radius_gain_;
    request.roll = fk->roll;
    request.pitch = 0.0;
    request.yaw = 0.0;
    request.kin_name = "ik";
    if (!std::isfinite(request.tar_x) || !std::isfinite(request.tar_y) ||
      !std::isfinite(request.tar_z))
    {
      throw std::runtime_error("Non-finite vendor IK target");
    }

    RCLCPP_INFO(
      get_logger(),
      "Calling vendor IK for orange target: world=(%.3f, %.3f, %.3f) adjusted_z=%.3f "
      "roll=%.3f.",
      world_target[0], world_target[1], world_target[2], request.tar_z, request.roll);

    const auto response = call_kinematics(request, "ik");
    if (!response.has_value()) {
      return std::nullopt;
    }

    RCLCPP_INFO(
      get_logger(),
      "Vendor IK raw response: joint1=%.3f joint2=%.3f joint3=%.3f joint4=%.3f "
      "joint5=%.3f joint6=%.3f pose=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f).",
      response->joint1, response->joint2, response->joint3, response->joint4,
      response->joint5, response->joint6, response->x, response->y, response->z,
      response->roll, response->pitch, response->yaw);

    ServoPose pose{};
    pose[0] = validate_servo_angle(0U, response->joint1);
    pose[1] = validate_servo_angle(1U, response->joint2);
    pose[2] = validate_servo_angle(2U, response->joint3);
    pose[3] = std::min(validate_servo_angle(3U, response->joint4), ik_joint4_max_deg_);
    pose[4] = validate_servo_angle(4U, use_ik_joint5_ ? response->joint5 : ik_wrist_angle_deg_);
    pose[5] = gripper_open_angle_deg_;

    RCLCPP_INFO(
      get_logger(),
      "Vendor IK orange grasp pose: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg.",
      pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    return pose;
  }

  std::vector<ServoStep> build_ik_grasp_steps(const ServoPose & ik_pose) const
  {
    ServoPose reach_open = ik_pose;
    reach_open[5] = gripper_open_angle_deg_;

    ServoPose grasp_closed = ik_pose;
    grasp_closed[5] = gripper_close_angle_deg_;

    ServoPose lift = grasp_closed;
    lift[1] = ik_lift_servo2_angle_deg_;

    std::vector<ServoStep> steps{
      {"vendor ik reach/open", reach_open, pregrasp_motion_time_ms_, default_settle_ms_},
      {"vendor close gripper", grasp_closed, grip_motion_time_ms_, default_settle_ms_},
      {"vendor lift/hold", lift, lift_motion_time_ms_, default_settle_ms_},
    };
    if (return_to_finish_pose_) {
      ServoPose finish = finish_servo_angles_deg_;
      finish[5] = gripper_close_angle_deg_;
      steps.push_back({"finish/hold", finish, finish_motion_time_ms_, default_settle_ms_});
    }
    return steps;
  }

  std::optional<Kinemarics::Response> call_fk(const ServoPose & pose)
  {
    Kinemarics::Request request;
    request.cur_joint1 = pose[0];
    request.cur_joint2 = pose[1];
    request.cur_joint3 = pose[2];
    request.cur_joint4 = pose[3];
    request.cur_joint5 = pose[4];
    request.cur_joint6 = pose[5];
    request.kin_name = "fk";
    return call_kinematics(request, "fk");
  }

  std::optional<Kinemarics::Response> call_kinematics(
    const Kinemarics::Request & request,
    const std::string & label)
  {
    if (!kinematics_client_) {
      RCLCPP_ERROR(get_logger(), "Vendor kinematics client is not configured.");
      return std::nullopt;
    }

    const auto timeout = std::chrono::duration<double>(kinematics_wait_sec_);
    if (!kinematics_client_->wait_for_service(timeout)) {
      RCLCPP_ERROR(
        get_logger(), "Timed out waiting for vendor kinematics service '%s'.",
        kinematics_service_name_.c_str());
      return std::nullopt;
    }

    auto request_ptr = std::make_shared<Kinemarics::Request>(request);
    auto future = kinematics_client_->async_send_request(request_ptr);
    if (future.wait_for(timeout) != std::future_status::ready) {
      kinematics_client_->remove_pending_request(future);
      RCLCPP_ERROR(get_logger(), "Timed out waiting for vendor %s response.", label.c_str());
      return std::nullopt;
    }

    const auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(get_logger(), "Vendor %s service returned an empty response.", label.c_str());
      return std::nullopt;
    }
    const std::array<double, 6> values = label == "fk" ?
      std::array<double, 6>{response->x, response->y, response->z,
        response->roll, response->pitch, response->yaw} :
      std::array<double, 6>{response->joint1, response->joint2, response->joint3,
        response->joint4, response->joint5, response->joint6};
    if (!std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);})) {
      throw std::runtime_error("Non-finite vendor kinematics response");
    }
    return *response;
  }

  std::array<double, 3> target_to_vendor_world(
    const geometry_msgs::msg::PointStamped & target,
    const Kinemarics::Response & current_end_pose) const
  {
    std::array<double, 3> point{{target.point.x, target.point.y, target.point.z}};
    if (target_point_mode_ == "camera_optical") {
      point = transform_point(vendor_end_to_camera(), point);
      point = transform_point(endpoint_transform(current_end_pose), point);
    }

    point[0] += target_world_offset_x_m_;
    point[1] += target_world_offset_y_m_;
    point[2] += target_world_offset_z_m_;
    return point;
  }

  double target_yaw_deg(const geometry_msgs::msg::PointStamped & target) const
  {
    if (!apply_target_yaw_) {
      return pregrasp_servo_angles_deg_[0];
    }

    const double commanded_yaw =
      pregrasp_servo_angles_deg_[0] + target_yaw_gain_ * target_lateral_yaw_error_deg(target) +
      target_yaw_offset_deg_;
    return std::clamp(commanded_yaw, target_yaw_min_deg_, target_yaw_max_deg_);
  }

  double target_lateral_yaw_error_deg(const geometry_msgs::msg::PointStamped & target) const
  {
    const bool camera_mode = target_point_mode_ == "camera_optical";
    const double lateral = camera_mode ? target.point.x : target.point.y;
    const double depth = camera_mode ? target.point.z : target.point.x;
    return std::atan2(lateral, depth) * 180.0 / kPi;
  }

  ServoPose pose_with_yaw_and_gripper(
    ServoPose pose,
    double yaw_deg,
    double gripper_deg) const
  {
    pose[0] = validate_servo_angle(0U, yaw_deg);
    pose[5] = validate_servo_angle(5U, gripper_deg);
    return pose;
  }

  static std::array<double, 3> transform_point(
    const RigidTransform & transform,
    const std::array<double, 3> & point)
  {
    std::array<double, 3> result{};
    for (std::size_t row = 0; row < result.size(); ++row) {
      result[row] = transform.translation[row];
      for (std::size_t col = 0; col < point.size(); ++col) {
        result[row] += transform.rotation.data[row][col] * point[col];
      }
    }
    return result;
  }

  static RigidTransform endpoint_transform(const Kinemarics::Response & pose)
  {
    RigidTransform transform;
    transform.translation = {pose.x, pose.y, pose.z};
    transform.rotation = rotation_from_rpy(pose.roll, pose.pitch, pose.yaw);
    return transform;
  }

  static Matrix3 rotation_from_rpy(double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    Matrix3 matrix;
    matrix.data = {{
      {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr}},
      {{sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr}},
      {{-sp, cp * sr, cp * cr}},
    }};
    return matrix;
  }

  static const RigidTransform & vendor_end_to_camera()
  {
    static const RigidTransform transform = []() {
      RigidTransform value;
      value.rotation.data = {{
        {{1.0, 0.0, 0.0}},
        {{0.0, 7.96326711e-04, 9.99999683e-01}},
        {{0.0, -9.99999683e-01, 7.96326711e-04}},
      }};
      value.translation = {0.0, -0.099, 0.049};
      return value;
    }();
    return transform;
  }

  bool execute_steps(
    edgepick_hardware::DofbotI2cTransport & transport,
    const std::vector<ServoStep> & steps) const
  {
    for (const auto & step : steps) {
      if (stop_requested_ || !rclcpp::ok()) {
        return false;
      }
      if (!send_pose(transport, step)) {
        return false;
      }
    }
    return true;
  }

  bool send_pose(
    edgepick_hardware::DofbotI2cTransport & transport,
    const ServoStep & step) const
  {
    if (stop_requested_ || !rclcpp::ok()) {
      return false;
    }
    edgepick_hardware::JointCommand command;
    command.angles_deg = step.pose;
    command.motion_time = std::chrono::milliseconds{step.motion_time_ms};

    RCLCPP_INFO(
      get_logger(),
      "Sending direct orange grasp step '%s': [%.0f, %.0f, %.0f, %.0f, %.0f, %.0f] deg, "
      "time=%d ms.",
      step.label.c_str(), step.pose[0], step.pose[1], step.pose[2], step.pose[3], step.pose[4],
      step.pose[5], step.motion_time_ms);

    if (use_real_i2c_ && !transport.write(command)) {
      RCLCPP_ERROR(
        get_logger(), "Direct orange grasp step '%s' failed on %s at address 0x%02x.",
        step.label.c_str(), i2c_device_.c_str(), static_cast<unsigned int>(i2c_address_));
      return false;
    }

    if (!use_real_i2c_) {
      RCLCPP_INFO(get_logger(), "Direct orange grasp step '%s' dry-run complete.", step.label.c_str());
    }

    return settle(step.motion_time_ms + step.settle_time_ms);
  }

  void publish_event(TaskEvent event)
  {
    if (stop_requested_ || !rclcpp::ok()) {
      return;
    }
    std_msgs::msg::String message;
    message.data = to_string(event);
    event_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Published task event '%s'.", message.data.c_str());
  }

  bool settle(int settle_ms) const
  {
    if (settle_ms <= 0) {
      return !stop_requested_ && rclcpp::ok();
    }
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::milliseconds{settle_ms}, [this]() {
      return stop_requested_ || !rclcpp::ok();
    });
    return !stop_requested_ && rclcpp::ok();
  }

  int read_motion_time_ms(const std::string & name, int default_value)
  {
    return std::clamp(
      static_cast<int>(declare_parameter<int>(name, default_value)), 20, 30000);
  }

  ServoPose read_servo_pose(const std::string & prefix, const ServoPose & defaults)
  {
    ServoPose pose{};
    for (std::size_t index = 0; index < pose.size(); ++index) {
      pose[index] = validate_servo_angle(
        index, declare_parameter<double>(prefix + std::to_string(index + 1), defaults[index]));
    }
    return pose;
  }

  double validate_servo_angle(std::size_t index, double angle_deg) const
  {
    const double max_deg = index == 4U ? 270.0 : 180.0;
    if (!std::isfinite(angle_deg) || angle_deg < 0.0 || angle_deg > max_deg) {
      throw std::runtime_error(
        "servo angle " + std::to_string(index + 1) + " must be within 0.." +
        std::to_string(static_cast<int>(max_deg)) + " degrees");
    }
    return angle_deg;
  }

  std::uint8_t parse_i2c_address(const std::string & text) const
  {
    return edgepick_hardware::parse_i2c_address(text);
  }

  static constexpr double kPi = 3.14159265358979323846;

  std::string event_topic_;
  std::string state_topic_;
  std::string target_point_topic_;
  std::string startup_ready_topic_;
  bool require_startup_pose_ready_{true};
  bool startup_pose_ready_{false};
  bool use_real_i2c_{true};
  std::string i2c_device_;
  std::uint8_t i2c_address_{0x15};
  bool use_kinematics_service_{true};
  std::string kinematics_service_name_;
  double kinematics_wait_sec_{5.0};
  double state_transition_wait_sec_{10.0};
  double startup_wait_sec_{30.0};
  double target_wait_sec_{30.0};
  int verification_settle_ms_{300};
  int default_settle_ms_{250};
  int pregrasp_motion_time_ms_{1000};
  int descend_motion_time_ms_{1000};
  int grip_motion_time_ms_{600};
  int lift_motion_time_ms_{1000};
  int finish_motion_time_ms_{1000};
  double gripper_open_angle_deg_{30.0};
  double gripper_close_angle_deg_{142.0};
  bool apply_target_yaw_{true};
  double target_yaw_gain_{1.0};
  double target_yaw_offset_deg_{0.0};
  double target_yaw_min_deg_{20.0};
  double target_yaw_max_deg_{160.0};
  bool tracking_enabled_{false};
  int tracking_max_updates_{6};
  int tracking_centered_updates_{2};
  double tracking_center_tolerance_deg_{4.0};
  double tracking_yaw_gain_{0.65};
  int tracking_motion_time_ms_{250};
  int tracking_target_max_age_ms_{500};
  std::string target_point_mode_{"camera_optical"};
  std::string expected_target_frame_;
  double target_world_offset_x_m_{0.0};
  double target_world_offset_y_m_{0.0};
  double target_world_offset_z_m_{0.0};
  double ik_target_z_radius_origin_m_{0.181};
  double ik_target_z_radius_gain_{0.15};
  double ik_joint4_max_deg_{90.0};
  double ik_wrist_angle_deg_{90.0};
  bool use_ik_joint5_{false};
  double ik_lift_servo2_angle_deg_{120.0};
  double min_target_depth_m_{0.05};
  double max_target_depth_m_{1.20};
  double max_abs_target_lateral_m_{0.35};
  double min_target_vertical_m_{-0.35};
  double max_target_vertical_m_{0.35};
  bool return_to_finish_pose_{false};
  ServoPose pregrasp_servo_angles_deg_{};
  ServoPose grasp_servo_angles_deg_{};
  ServoPose lift_servo_angles_deg_{};
  ServoPose finish_servo_angles_deg_{};
  ServoPose ik_reference_servo_angles_deg_{};
  TaskState current_state_{TaskState::kIdle};
  std::optional<geometry_msgs::msg::PointStamped> latest_target_point_;
  std::optional<std::chrono::steady_clock::time_point> latest_target_received_at_;
  std::atomic<int> exit_code_{1};
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  std::atomic_bool stop_requested_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr startup_ready_subscription_;
  rclcpp::Client<Kinemarics>::SharedPtr kinematics_client_;
  std::thread worker_;
};

}  // namespace
}  // namespace edgepick_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<edgepick_task::OrangeGraspExecutorNode>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return node->exit_code();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "orange_grasp_executor_node failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
