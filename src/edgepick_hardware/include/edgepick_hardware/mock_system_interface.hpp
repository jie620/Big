#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <memory>
#include <string>
#include <vector>

#include "edgepick_hardware/command_gateway.hpp"
#include "edgepick_hardware/dofbot_servo_mapping.hpp"
#include "edgepick_hardware/mock_transport.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace edgepick_hardware
{

class DofbotI2cTransport;

// ros2_control-facing mock hardware for the DOFBOT control chain.
//
// This class adapts controller-manager position interfaces to the EdgePick
// command gateway. It is deliberately still a mock: it records writes through
// MockTransport and never opens I2C or moves the physical arm.
class MockSystemInterface final : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  const std::vector<std::string> & joint_names() const;
  const std::vector<MockWrite> & writes() const;
  const GatewayStatistics & gateway_statistics() const;
  std::optional<CommandStatus> last_write_status() const;

private:
  bool initialize_joint_storage(const hardware_interface::HardwareInfo & hardware_info);
  bool component_has_position_command(const hardware_interface::ComponentInfo & component) const;
  bool component_has_supported_state_interfaces(
    const hardware_interface::ComponentInfo & component) const;
  double initial_position_for(const hardware_interface::ComponentInfo & component) const;
  JointCommand build_command_from_ros_positions(const rclcpp::Duration & period) const;
  bool read_real_servo_state(bool sync_command_positions);
  std::chrono::steady_clock::time_point steady_time_from_ros_time(
    const rclcpp::Time & time) const;
  double seconds_from_period(const rclcpp::Duration & period) const;

  // These vectors back ros2_control's exported interface handles. Their
  // storage must outlive the handles returned by export_*_interfaces().
  std::vector<std::string> joint_names_;
  std::vector<double> state_positions_rad_;
  std::vector<double> state_velocities_rad_s_;
  std::vector<double> command_positions_rad_;

  // Trajectory points are streamed much faster than a one-shot vendor action.
  // A short duration prevents each new point from restarting a long servo move.
  std::chrono::milliseconds motion_time_{30};
  std::unique_ptr<CommandTransport> transport_;
  DofbotI2cTransport * i2c_transport_{nullptr};
  MockTransport * mock_transport_{nullptr};
  bool use_real_i2c_{false};
  bool feedback_failed_{false};
  unsigned int consecutive_read_failures_{0};
  std::chrono::steady_clock::time_point last_real_read_at_{};
  std::optional<CommandGateway> gateway_;
  std::optional<CommandStatus> last_write_status_;
};

}  // namespace edgepick_hardware
