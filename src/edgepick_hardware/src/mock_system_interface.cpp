#include "edgepick_hardware/mock_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "edgepick_hardware/dofbot_i2c_transport.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace edgepick_hardware
{
namespace
{

bool is_position_interface(const hardware_interface::InterfaceInfo & interface)
{
  return interface.name == hardware_interface::HW_IF_POSITION;
}

bool is_velocity_interface(const hardware_interface::InterfaceInfo & interface)
{
  return interface.name == hardware_interface::HW_IF_VELOCITY;
}

bool parse_bool_parameter(const std::string & value, bool default_value = false)
{
  if (value.empty()) {
    return default_value;
  }
  if (value == "1" || value == "true" || value == "TRUE" || value == "True" ||
      value == "yes" || value == "YES" || value == "on" || value == "ON")
  {
    return true;
  }
  if (value == "0" || value == "false" || value == "FALSE" || value == "False" ||
      value == "no" || value == "NO" || value == "off" || value == "OFF")
  {
    return false;
  }
  return default_value;
}

std::string parameter_or(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & key,
  const std::string & default_value)
{
  const auto iter = hardware_info.hardware_parameters.find(key);
  if (iter == hardware_info.hardware_parameters.end() || iter->second.empty()) {
    return default_value;
  }
  return iter->second;
}

}  // namespace

hardware_interface::CallbackReturn MockSystemInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!initialize_joint_storage(hardware_info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const bool use_real_i2c =
    parse_bool_parameter(parameter_or(hardware_info, "use_real_i2c", "false"));

  try {
    if (use_real_i2c) {
      DofbotI2cConfig i2c_config;
      i2c_config.enabled = true;
      i2c_config.device = parameter_or(hardware_info, "i2c_device", i2c_config.device);
      const auto address_text = parameter_or(hardware_info, "i2c_address", "0x15");
      i2c_config.address = static_cast<std::uint8_t>(std::stoul(address_text, nullptr, 0));
      transport_ = std::make_unique<DofbotI2cTransport>(i2c_config);
      mock_transport_ = nullptr;
    } else {
      auto mock_transport = std::make_unique<MockTransport>();
      mock_transport_ = mock_transport.get();
      transport_ = std::move(mock_transport);
    }
  } catch (const std::exception &) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Keep the gateway policy here so controller-manager traffic still passes
  // through the same command safety gate used by lower-level tests.
  GatewayConfig config;
  config.min_command_interval = std::chrono::milliseconds{20};
  gateway_.emplace(*transport_, config);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MockSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto & joint = info_.joints[index];
    for (const auto & state_interface : joint.state_interfaces) {
      if (is_position_interface(state_interface)) {
        state_interfaces.emplace_back(
          joint.name, hardware_interface::HW_IF_POSITION, &state_positions_rad_[index]);
      } else if (is_velocity_interface(state_interface)) {
        state_interfaces.emplace_back(
          joint.name, hardware_interface::HW_IF_VELOCITY, &state_velocities_rad_s_[index]);
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MockSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    command_interfaces.emplace_back(
      info_.joints[index].name,
      hardware_interface::HW_IF_POSITION,
      &command_positions_rad_[index]);
  }

  return command_interfaces;
}

hardware_interface::return_type MockSystemInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // No external hardware is sampled in the mock. State is updated in write()
  // when the command gateway accepts a command.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockSystemInterface::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  if (!gateway_.has_value()) {
    return hardware_interface::return_type::ERROR;
  }

  const JointCommand command = build_command_from_ros_positions();
  const CommandStatus status = gateway_->submit(command, steady_time_from_ros_time(time));
  last_write_status_ = status;

  if (status == CommandStatus::kAccepted) {
    // A successful mock write makes joint state follow the controller command.
    // This mirrors GenericSystem-style RViz behavior while still exercising the
    // EdgePick command validation path.
    const double period_seconds = seconds_from_period(period);
    for (std::size_t index = 0; index < command_positions_rad_.size(); ++index) {
      const double previous_position = state_positions_rad_[index];
      state_positions_rad_[index] = command_positions_rad_[index];
      state_velocities_rad_s_[index] =
        period_seconds > 0.0 ? (state_positions_rad_[index] - previous_position) / period_seconds :
        0.0;
    }
    return hardware_interface::return_type::OK;
  }

  if (status == CommandStatus::kDuplicateSuppressed || status == CommandStatus::kRateLimited) {
    // Soft rejections are expected during controller updates, so they should not
    // make controller_manager treat the hardware component as failed.
    std::fill(state_velocities_rad_s_.begin(), state_velocities_rad_s_.end(), 0.0);
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::ERROR;
}

const std::vector<std::string> & MockSystemInterface::joint_names() const
{
  return joint_names_;
}

const std::vector<MockWrite> & MockSystemInterface::writes() const
{
  static const std::vector<MockWrite> empty_writes;
  return mock_transport_ ? mock_transport_->writes() : empty_writes;
}

const GatewayStatistics & MockSystemInterface::gateway_statistics() const
{
  if (!gateway_.has_value()) {
    static const GatewayStatistics empty_statistics;
    return empty_statistics;
  }
  return gateway_->statistics();
}

std::optional<CommandStatus> MockSystemInterface::last_write_status() const
{
  return last_write_status_;
}

bool MockSystemInterface::initialize_joint_storage(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // The current DOFBOT control model is five arm joints plus one gripper joint.
  // Rejecting other layouts makes xacro/controller mismatches fail early.
  if (hardware_info.joints.size() != kJointCount) {
    return false;
  }

  joint_names_.clear();
  joint_names_.reserve(kJointCount);
  state_positions_rad_.assign(kJointCount, 0.0);
  state_velocities_rad_s_.assign(kJointCount, 0.0);
  command_positions_rad_.assign(kJointCount, 0.0);

  for (std::size_t index = 0; index < hardware_info.joints.size(); ++index) {
    const auto & joint = hardware_info.joints[index];
    if (!component_has_position_command(joint) || !component_has_supported_state_interfaces(joint)) {
      return false;
    }

    const double initial_position = initial_position_for(joint);
    joint_names_.push_back(joint.name);
    state_positions_rad_[index] = initial_position;
    command_positions_rad_[index] = initial_position;
  }

  return true;
}

bool MockSystemInterface::component_has_position_command(
  const hardware_interface::ComponentInfo & component) const
{
  return component.command_interfaces.size() == 1U &&
         is_position_interface(component.command_interfaces.front());
}

bool MockSystemInterface::component_has_supported_state_interfaces(
  const hardware_interface::ComponentInfo & component) const
{
  const bool has_position_state = std::any_of(
    component.state_interfaces.begin(), component.state_interfaces.end(), is_position_interface);
  const bool all_supported = std::all_of(
    component.state_interfaces.begin(), component.state_interfaces.end(),
    [](const hardware_interface::InterfaceInfo & interface) {
      return is_position_interface(interface) || is_velocity_interface(interface);
    });

  return has_position_state && all_supported;
}

double MockSystemInterface::initial_position_for(
  const hardware_interface::ComponentInfo & component) const
{
  const auto state_interface = std::find_if(
    component.state_interfaces.begin(), component.state_interfaces.end(), is_position_interface);
  if (state_interface == component.state_interfaces.end() || state_interface->initial_value.empty()) {
    return 0.0;
  }

  try {
    return std::stod(state_interface->initial_value);
  } catch (const std::exception &) {
    return 0.0;
  }
}

JointCommand MockSystemInterface::build_command_from_ros_positions() const
{
  JointCommand command;
  command.motion_time = motion_time_;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    command.angles_deg[index] = ros_position_to_command_degrees(index, command_positions_rad_[index]);
  }
  return command;
}

double MockSystemInterface::ros_position_to_command_degrees(
  std::size_t index,
  double position_rad) const
{
  // Linear mapping is sufficient for mock control-chain validation. Real I2C
  // enablement must replace or verify this with measured joint calibration.
  const auto & calibration = kDefaultCalibration[index];
  const double ros_span = calibration.ros_max_rad - calibration.ros_min_rad;
  const double command_span = calibration.command_max_deg - calibration.command_min_deg;
  return calibration.command_min_deg +
         ((position_rad - calibration.ros_min_rad) / ros_span) * command_span;
}

std::chrono::steady_clock::time_point MockSystemInterface::steady_time_from_ros_time(
  const rclcpp::Time & time) const
{
  // CommandGateway only needs monotonic spacing, not wall-clock semantics.
  // Tests can therefore use deterministic rclcpp::Time values.
  return std::chrono::steady_clock::time_point{std::chrono::nanoseconds{time.nanoseconds()}};
}

double MockSystemInterface::seconds_from_period(const rclcpp::Duration & period) const
{
  const auto nanoseconds = period.nanoseconds();
  if (nanoseconds <= 0) {
    return 0.0;
  }
  return static_cast<double>(nanoseconds) / 1000000000.0;
}

}  // namespace edgepick_hardware

PLUGINLIB_EXPORT_CLASS(edgepick_hardware::MockSystemInterface, hardware_interface::SystemInterface)
