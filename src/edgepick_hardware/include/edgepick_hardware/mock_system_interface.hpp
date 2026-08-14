#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "edgepick_hardware/command_gateway.hpp"
#include "edgepick_hardware/mock_transport.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace edgepick_hardware
{

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
  // URDF/MoveIt commands are radians, while the DOFBOT servo protocol uses
  // degrees. These ranges mirror the vendor URDF and stay centralized so the
  // future real hardware backend has one obvious calibration point.
  struct JointCalibration
  {
    double ros_min_rad;
    double ros_max_rad;
    double command_min_deg;
    double command_max_deg;
  };

  bool initialize_joint_storage(const hardware_interface::HardwareInfo & hardware_info);
  bool component_has_position_command(const hardware_interface::ComponentInfo & component) const;
  bool component_has_supported_state_interfaces(
    const hardware_interface::ComponentInfo & component) const;
  double initial_position_for(const hardware_interface::ComponentInfo & component) const;
  JointCommand build_command_from_ros_positions() const;
  double ros_position_to_command_degrees(std::size_t index, double position_rad) const;
  std::chrono::steady_clock::time_point steady_time_from_ros_time(
    const rclcpp::Time & time) const;
  double seconds_from_period(const rclcpp::Duration & period) const;

  static constexpr std::array<JointCalibration, kJointCount> kDefaultCalibration{{
    {-1.5708, 1.5708, 0.0, 180.0},
    {-1.5708, 1.5708, 0.0, 180.0},
    {-1.5708, 1.5708, 0.0, 180.0},
    {-1.5708, 1.5708, 0.0, 180.0},
    {-1.5708, 3.1416, 0.0, 270.0},
    {-1.6, 0.0, 0.0, 180.0},
  }};

  // These vectors back ros2_control's exported interface handles. Their
  // storage must outlive the handles returned by export_*_interfaces().
  std::vector<std::string> joint_names_;
  std::vector<double> state_positions_rad_;
  std::vector<double> state_velocities_rad_s_;
  std::vector<double> command_positions_rad_;

  // Mock writes use a fixed nominal movement time until the bringup layer
  // exposes trajectory timing or parameters.
  std::chrono::milliseconds motion_time_{1000};
  MockTransport transport_;
  std::optional<CommandGateway> gateway_;
  std::optional<CommandStatus> last_write_status_;
};

}  // namespace edgepick_hardware
