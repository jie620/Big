#include "edgepick_hardware/dofbot_servo_mapping.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace edgepick_hardware
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

void validate_index(std::size_t index)
{
  if (index >= kJointCount) {
    throw std::out_of_range("DOFBOT joint index must be 0..5");
  }
}

}  // namespace

double ros_position_to_servo_degree(std::size_t index, double position_rad)
{
  validate_index(index);
  const double position_deg = position_rad * kRadToDeg;

  if (index < 5U) {
    // Vendor dofbot_pro_driver maps MoveIt radians to Arm_Lib degrees as:
    // servo[1..5] = degrees(rad) + [90, 90, 90, 90, 90].
    return position_deg + 90.0;
  }

  // Vendor gripper command path maps MoveIt open (near 0 rad) to servo 30
  // and MoveIt closed (near -pi/2 rad) to servo 180.
  return 30.0 - position_deg * (150.0 / 90.0);
}

double servo_degree_to_ros_position(std::size_t index, double angle_deg)
{
  validate_index(index);

  if (index < 5U) {
    return (angle_deg - 90.0) * kDegToRad;
  }

  return ((30.0 - angle_deg) * (90.0 / 150.0)) * kDegToRad;
}

ServoAngles ros_positions_to_servo_degrees(const RosPositions & positions_rad)
{
  ServoAngles angles_deg{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    angles_deg[index] = ros_position_to_servo_degree(index, positions_rad[index]);
  }
  return angles_deg;
}

RosPositions servo_degrees_to_ros_positions(const ServoAngles & angles_deg)
{
  RosPositions positions_rad{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    positions_rad[index] = servo_degree_to_ros_position(index, angles_deg[index]);
  }
  return positions_rad;
}

}  // namespace edgepick_hardware
