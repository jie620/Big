#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "edgepick_hardware/command.hpp"
#include "edgepick_hardware/dofbot_i2c_transport.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace edgepick_task
{

class StartupPoseSequenceNode final : public rclcpp::Node
{
public:
  StartupPoseSequenceNode()
  : Node("edgepick_startup_pose_sequence")
  {
    use_real_i2c_ = declare_parameter<bool>("use_real_i2c", true);
    i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-7");
    i2c_address_ = parse_i2c_address(
      declare_parameter<std::string>("i2c_address", "0x15"));
    ready_topic_ =
      declare_parameter<std::string>("ready_topic", "/edgepick/startup_pose/ready");
    zero_motion_time_ms_ = read_motion_time_ms("zero_motion_time_ms", 3000);
    restore_motion_time_ms_ = read_motion_time_ms("restore_motion_time_ms", 2000);
    settle_time_ms_ = std::max(
      0, static_cast<int>(declare_parameter<int>("settle_time_ms", 250)));

    zero_servo_angles_deg_ = read_servo_pose(
      "zero_servo_angle", std::array<double, edgepick_hardware::kJointCount>{
        {90.0, 90.0, 90.0, 90.0, 90.0, 30.0}});
    restore_servo_angles_deg_ = read_servo_pose(
      "restore_servo_angle", std::array<double, edgepick_hardware::kJointCount>{
        {90.0, 165.0, 18.0, 0.0, 90.0, 30.0}});

    ready_publisher_ = create_publisher<std_msgs::msg::String>(
      ready_topic_, rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(
      get_logger(),
      "Startup pose sequence ready: direct I2C zero/home -> fixed restore, device=%s "
      "address=0x%02x",
      i2c_device_.c_str(), static_cast<unsigned int>(i2c_address_));
  }

  int run()
  {
    try {
      edgepick_hardware::DofbotI2cConfig config;
      config.enabled = use_real_i2c_;
      config.device = i2c_device_;
      config.address = i2c_address_;
      edgepick_hardware::DofbotI2cTransport transport(config);

      if (!send_pose(
          transport, "zero/home", zero_servo_angles_deg_, zero_motion_time_ms_))
      {
        return 1;
      }
      if (!send_pose(
          transport, "fixed restore", restore_servo_angles_deg_, restore_motion_time_ms_))
      {
        return 1;
      }

      std_msgs::msg::String ready;
      ready.data = "ready";
      ready_publisher_->publish(ready);
      RCLCPP_INFO(
        get_logger(),
        "Startup pose sequence completed: zero/home then [90, 165, 18, 0, 90, 30].");
      return 0;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Startup pose sequence failed: %s", error.what());
      return 1;
    }
  }

private:
  int read_motion_time_ms(const std::string & name, int default_value)
  {
    return std::clamp(
      static_cast<int>(declare_parameter<int>(name, default_value)), 20, 30000);
  }

  std::array<double, edgepick_hardware::kJointCount> read_servo_pose(
    const std::string & prefix,
    const std::array<double, edgepick_hardware::kJointCount> & defaults)
  {
    std::array<double, edgepick_hardware::kJointCount> pose{};
    for (std::size_t index = 0; index < pose.size(); ++index) {
      pose[index] = declare_parameter<double>(
        prefix + std::to_string(index + 1), defaults[index]);
      validate_servo_angle(prefix, index, pose[index]);
    }
    return pose;
  }

  void validate_servo_angle(
    const std::string & prefix,
    std::size_t index,
    double angle_deg) const
  {
    const double max_deg = index == 4U ? 270.0 : 180.0;
    if (!std::isfinite(angle_deg) || angle_deg < 0.0 || angle_deg > max_deg) {
      throw std::runtime_error(
        prefix + std::to_string(index + 1) + " must be within 0.." +
        std::to_string(static_cast<int>(max_deg)) + " degrees");
    }
  }

  std::uint8_t parse_i2c_address(const std::string & text) const
  {
    try {
      const auto parsed = std::stoul(text, nullptr, 0);
      if (parsed > std::numeric_limits<std::uint8_t>::max()) {
        throw std::out_of_range("I2C address out of uint8 range");
      }
      return static_cast<std::uint8_t>(parsed);
    } catch (const std::exception & error) {
      throw std::runtime_error("invalid i2c_address '" + text + "': " + error.what());
    }
  }

  bool send_pose(
    edgepick_hardware::DofbotI2cTransport & transport,
    const std::string & label,
    const std::array<double, edgepick_hardware::kJointCount> & angles_deg,
    int motion_time_ms) const
  {
    edgepick_hardware::JointCommand command;
    command.angles_deg = angles_deg;
    command.motion_time = std::chrono::milliseconds{motion_time_ms};

    RCLCPP_INFO(
      get_logger(),
      "Sending startup %s servo pose: [%.0f, %.0f, %.0f, %.0f, %.0f, %.0f] deg, "
      "time=%d ms.",
      label.c_str(), angles_deg[0], angles_deg[1], angles_deg[2], angles_deg[3],
      angles_deg[4], angles_deg[5], motion_time_ms);

    if (use_real_i2c_ && !transport.write(command)) {
      RCLCPP_ERROR(
        get_logger(), "Startup %s servo pose failed on %s at address 0x%02x.",
        label.c_str(), i2c_device_.c_str(), static_cast<unsigned int>(i2c_address_));
      return false;
    }

    if (!use_real_i2c_) {
      RCLCPP_INFO(get_logger(), "Startup %s servo pose dry-run complete.", label.c_str());
    }

    rclcpp::sleep_for(std::chrono::milliseconds{motion_time_ms + settle_time_ms_});
    return true;
  }

  bool use_real_i2c_{true};
  std::string i2c_device_;
  std::uint8_t i2c_address_{0x15};
  std::string ready_topic_;
  int zero_motion_time_ms_{3000};
  int restore_motion_time_ms_{2000};
  int settle_time_ms_{250};
  std::array<double, edgepick_hardware::kJointCount> zero_servo_angles_deg_{};
  std::array<double, edgepick_hardware::kJointCount> restore_servo_angles_deg_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ready_publisher_;
};

}  // namespace edgepick_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<edgepick_task::StartupPoseSequenceNode>();
  const int exit_code = node->run();
  rclcpp::shutdown();
  return exit_code;
}
