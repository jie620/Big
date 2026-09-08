#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "edgepick_hardware/command.hpp"
#include "edgepick_hardware/dofbot_i2c_transport.hpp"

#include <rclcpp/rclcpp.hpp>

namespace edgepick_task
{

class ServoPoseCommandNode final : public rclcpp::Node
{
public:
  ServoPoseCommandNode()
  : Node("edgepick_servo_pose_command")
  {
    use_real_i2c_ = declare_parameter<bool>("use_real_i2c", true);
    i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-7");
    i2c_address_ = edgepick_hardware::parse_i2c_address(
      std::to_string(declare_parameter<int64_t>("i2c_address", 0x15)));
    motion_time_ms_ = std::clamp(
      static_cast<int>(declare_parameter<int>("motion_time_ms", 2000)), 20, 30000);

    const std::vector<double> default_pose{90.0, 165.0, 18.0, 0.0, 90.0, 30.0};
    const auto pose = declare_parameter<std::vector<double>>("servo_angles", default_pose);
    if (pose.size() != edgepick_hardware::kJointCount) {
      throw std::runtime_error("servo_angles must contain exactly 6 values");
    }
    for (std::size_t index = 0; index < edgepick_hardware::kJointCount; ++index) {
      servo_angles_deg_[index] = pose[index];
      const double max_deg = index == 4U ? 270.0 : 180.0;
      if (!std::isfinite(servo_angles_deg_[index]) || servo_angles_deg_[index] < 0.0 || servo_angles_deg_[index] > max_deg) {
        throw std::runtime_error("servo_angles contains a value outside the DOFBOT range");
      }
    }
  }

  int run()
  {
    edgepick_hardware::DofbotI2cConfig config;
    config.enabled = use_real_i2c_;
    config.device = i2c_device_;
    config.address = i2c_address_;
    edgepick_hardware::DofbotI2cTransport transport(config);

    edgepick_hardware::JointCommand command;
    command.angles_deg = servo_angles_deg_;
    command.motion_time = std::chrono::milliseconds{motion_time_ms_};

    RCLCPP_INFO(
      get_logger(),
      "Sending DOFBOT servo pose: [%.0f, %.0f, %.0f, %.0f, %.0f, %.0f] deg, time=%d ms.",
      servo_angles_deg_[0], servo_angles_deg_[1], servo_angles_deg_[2], servo_angles_deg_[3],
      servo_angles_deg_[4], servo_angles_deg_[5], motion_time_ms_);

    if (use_real_i2c_ && !transport.write(command)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to write DOFBOT servo pose on %s at address 0x%02x.",
        i2c_device_.c_str(), static_cast<unsigned int>(i2c_address_));
      return 1;
    }

    if (!use_real_i2c_) {
      RCLCPP_INFO(get_logger(), "DOFBOT servo pose dry-run complete.");
    }
    rclcpp::sleep_for(std::chrono::milliseconds{motion_time_ms_});
    return 0;
  }

private:
  bool use_real_i2c_{true};
  std::string i2c_device_;
  std::uint8_t i2c_address_{0x15};
  int motion_time_ms_{2000};
  std::array<double, edgepick_hardware::kJointCount> servo_angles_deg_{};
};

}  // namespace edgepick_task

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<edgepick_task::ServoPoseCommandNode>();
    const int exit_code = node->run();
    rclcpp::shutdown();
    return exit_code;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "servo_pose_command_node failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
