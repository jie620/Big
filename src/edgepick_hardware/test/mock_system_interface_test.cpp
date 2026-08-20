#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "edgepick_hardware/mock_system_interface.hpp"

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace edgepick_hardware
{
namespace
{

hardware_interface::InterfaceInfo interface_info(
  const std::string & name,
  const std::string & initial_value = {})
{
  hardware_interface::InterfaceInfo interface;
  interface.name = name;
  interface.initial_value = initial_value;
  return interface;
}

hardware_interface::HardwareInfo valid_hardware_info()
{
  hardware_interface::HardwareInfo info;
  info.name = "EdgePickMockSystem";
  info.type = "system";
  info.hardware_class_type = "edgepick_hardware/MockSystemInterface";

  const std::vector<std::string> joint_names{
    "Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint", "grip_joint"};

  for (const auto & joint_name : joint_names) {
    hardware_interface::ComponentInfo joint;
    joint.name = joint_name;
    joint.type = "joint";
    joint.command_interfaces.push_back(interface_info(hardware_interface::HW_IF_POSITION));
    joint.state_interfaces.push_back(interface_info(hardware_interface::HW_IF_POSITION, "0.0"));
    joint.state_interfaces.push_back(interface_info(hardware_interface::HW_IF_VELOCITY));
    info.joints.push_back(joint);
  }

  return info;
}

hardware_interface::HardwareInfo real_i2c_hardware_info()
{
  auto info = valid_hardware_info();
  info.hardware_parameters["use_real_i2c"] = "true";
  info.hardware_parameters["i2c_device"] = "/tmp/edgepick_missing_i2c_device";
  info.hardware_parameters["i2c_address"] = "0x15";
  return info;
}

rclcpp::Time ros_time_ms(int64_t milliseconds)
{
  return rclcpp::Time{milliseconds * 1000000};
}

rclcpp::Duration period_ms(int64_t milliseconds)
{
  return rclcpp::Duration{std::chrono::milliseconds{milliseconds}};
}

TEST(MockSystemInterfaceTest, RejectsConfigurationsWithoutSixJoints)
{
  auto info = valid_hardware_info();
  info.joints.pop_back();
  MockSystemInterface system;

  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST(MockSystemInterfaceTest, ExportsPositionCommandsAndPositionVelocityStates)
{
  MockSystemInterface system;
  ASSERT_EQ(system.on_init(valid_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);

  auto state_interfaces = system.export_state_interfaces();
  auto command_interfaces = system.export_command_interfaces();

  ASSERT_EQ(state_interfaces.size(), 12U);
  ASSERT_EQ(command_interfaces.size(), 6U);
  EXPECT_EQ(state_interfaces[0].get_name(), "Arm1_Joint/position");
  EXPECT_EQ(state_interfaces[1].get_name(), "Arm1_Joint/velocity");
  EXPECT_EQ(command_interfaces[0].get_name(), "Arm1_Joint/position");
}

TEST(MockSystemInterfaceTest, WritesValidCommandsThroughGatewayAndUpdatesState)
{
  MockSystemInterface system;
  ASSERT_EQ(system.on_init(valid_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  auto state_interfaces = system.export_state_interfaces();
  auto command_interfaces = system.export_command_interfaces();

  command_interfaces[0].set_value(0.1);

  EXPECT_EQ(
    system.write(ros_time_ms(100), period_ms(100)),
    hardware_interface::return_type::OK);
  ASSERT_EQ(system.writes().size(), 1U);
  EXPECT_EQ(system.last_write_status(), CommandStatus::kAccepted);
  EXPECT_NEAR(system.writes().front().command.angles_deg[0], 95.72956455309398, 1e-6);
  EXPECT_NEAR(state_interfaces[0].get_value(), 0.1, 1e-12);
  EXPECT_NEAR(state_interfaces[1].get_value(), 1.0, 1e-12);
}

TEST(MockSystemInterfaceTest, ConvertsDofbotMoveItRangesToServoDegrees)
{
  MockSystemInterface system;
  ASSERT_EQ(system.on_init(valid_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  auto command_interfaces = system.export_command_interfaces();

  command_interfaces[0].set_value(-1.5708);
  command_interfaces[4].set_value(3.1416);
  command_interfaces[5].set_value(-1.6);

  EXPECT_EQ(
    system.write(ros_time_ms(100), period_ms(100)),
    hardware_interface::return_type::OK);
  ASSERT_EQ(system.writes().size(), 1U);
  const auto & command = system.writes().front().command;
  EXPECT_NEAR(command.angles_deg[0], 0.0, 1e-9);
  EXPECT_NEAR(command.angles_deg[4], 270.0, 1e-9);
  EXPECT_NEAR(command.angles_deg[5], 0.0, 1e-9);
}

TEST(MockSystemInterfaceTest, RateLimitedCommandsDoNotBecomeHardwareErrors)
{
  MockSystemInterface system;
  ASSERT_EQ(system.on_init(valid_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  auto state_interfaces = system.export_state_interfaces();
  auto command_interfaces = system.export_command_interfaces();

  ASSERT_EQ(
    system.write(ros_time_ms(100), period_ms(100)),
    hardware_interface::return_type::OK);
  command_interfaces[0].set_value(0.2);

  EXPECT_EQ(
    system.write(ros_time_ms(110), period_ms(10)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(system.last_write_status(), CommandStatus::kRateLimited);
  EXPECT_EQ(system.writes().size(), 1U);
  EXPECT_NEAR(state_interfaces[0].get_value(), 0.0, 1e-12);
}

TEST(MockSystemInterfaceTest, OutOfRangeCommandsReturnErrorWithoutTransportWrite)
{
  MockSystemInterface system;
  ASSERT_EQ(system.on_init(valid_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  auto command_interfaces = system.export_command_interfaces();

  command_interfaces[0].set_value(2.0);

  EXPECT_EQ(
    system.write(ros_time_ms(100), period_ms(100)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.last_write_status(), CommandStatus::kInvalidJointAngle);
  EXPECT_TRUE(system.writes().empty());
}

TEST(MockSystemInterfaceTest, ExplicitRealI2cEnablementRejectsMissingDevice)
{
  MockSystemInterface system;
  EXPECT_EQ(system.on_init(real_i2c_hardware_info()), hardware_interface::CallbackReturn::ERROR);
}

}  // namespace
}  // namespace edgepick_hardware
