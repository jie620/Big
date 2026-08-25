#pragma once

#include <array>
#include <cstddef>

#include "edgepick_hardware/command.hpp"

namespace edgepick_hardware
{

// Vendor-facing DOFBOT pose: Arm_Lib/Arm_serial_servo_write6 degrees.
// Joints 1-4 and 6 use 0..180 degrees; joint 5 uses 0..270 degrees.
using ServoAngles = std::array<double, kJointCount>;

// ROS/MoveIt-facing pose: URDF joint positions in radians.
using RosPositions = std::array<double, kJointCount>;

double ros_position_to_servo_degree(std::size_t index, double position_rad);
double servo_degree_to_ros_position(std::size_t index, double angle_deg);

ServoAngles ros_positions_to_servo_degrees(const RosPositions & positions_rad);
RosPositions servo_degrees_to_ros_positions(const ServoAngles & angles_deg);

}  // namespace edgepick_hardware
