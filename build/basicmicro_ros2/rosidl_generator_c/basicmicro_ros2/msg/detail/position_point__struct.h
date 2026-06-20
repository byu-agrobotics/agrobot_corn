// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:msg/PositionPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/msg/position_point.h"


#ifndef BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__STRUCT_H_
#define BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/PositionPoint in the package basicmicro_ros2.
/**
  * Position point for servo position sequence execution
  * Simplified interface for absolute position control
 */
typedef struct basicmicro_ros2__msg__PositionPoint
{
  /// radians (absolute wheel position)
  double left_position;
  /// radians (absolute wheel position)
  double right_position;
  /// m/s (maximum speed during movement)
  double max_speed;
  /// m/s² (acceleration rate)
  double acceleration;
  /// m/s² (deceleration rate)
  double deceleration;
} basicmicro_ros2__msg__PositionPoint;

// Struct for a sequence of basicmicro_ros2__msg__PositionPoint.
typedef struct basicmicro_ros2__msg__PositionPoint__Sequence
{
  basicmicro_ros2__msg__PositionPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__msg__PositionPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__STRUCT_H_
