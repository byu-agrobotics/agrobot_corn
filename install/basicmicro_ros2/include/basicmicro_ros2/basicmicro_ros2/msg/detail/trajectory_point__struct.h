// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:msg/TrajectoryPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/msg/trajectory_point.h"


#ifndef BASICMICRO_ROS2__MSG__DETAIL__TRAJECTORY_POINT__STRUCT_H_
#define BASICMICRO_ROS2__MSG__DETAIL__TRAJECTORY_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'command_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TrajectoryPoint in the package basicmicro_ros2.
/**
  * Trajectory point for mixed distance/position trajectory execution
  * Supports both relative distance movements and absolute position movements
 */
typedef struct basicmicro_ros2__msg__TrajectoryPoint
{
  /// Command type - determines which fields are used
  /// 'distance' or 'position'
  rosidl_runtime_c__String command_type;
  /// For distance commands (relative movement)
  /// meters (relative distance to move)
  double left_distance;
  /// meters (relative distance to move)
  double right_distance;
  /// For position commands (absolute positioning - servo mode)
  /// radians (absolute wheel position)
  double left_position;
  /// radians (absolute wheel position)
  double right_position;
  /// m/s² (deceleration rate for position commands)
  double deceleration;
  /// Common parameters for both command types
  /// m/s (max speed during movement)
  double speed;
  /// m/s² (acceleration rate)
  double acceleration;
  /// seconds (optional timing constraint, 0 = no constraint)
  double duration;
} basicmicro_ros2__msg__TrajectoryPoint;

// Struct for a sequence of basicmicro_ros2__msg__TrajectoryPoint.
typedef struct basicmicro_ros2__msg__TrajectoryPoint__Sequence
{
  basicmicro_ros2__msg__TrajectoryPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__msg__TrajectoryPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__MSG__DETAIL__TRAJECTORY_POINT__STRUCT_H_
