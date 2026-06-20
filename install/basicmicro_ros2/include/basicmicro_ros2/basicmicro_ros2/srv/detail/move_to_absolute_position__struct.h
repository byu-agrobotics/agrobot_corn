// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:srv/MoveToAbsolutePosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/move_to_absolute_position.h"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__MOVE_TO_ABSOLUTE_POSITION__STRUCT_H_
#define BASICMICRO_ROS2__SRV__DETAIL__MOVE_TO_ABSOLUTE_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/MoveToAbsolutePosition in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__MoveToAbsolutePosition_Request
{
  /// Request
  /// Absolute position for left motor (radians)
  double left_position_radians;
  /// Absolute position for right motor (radians)
  double right_position_radians;
  /// Maximum speed during move (m/s)
  double max_speed;
  /// Acceleration rate (m/s²)
  double acceleration;
  /// Deceleration rate (m/s²)
  double deceleration;
  /// Whether to buffer the command for chaining
  bool buffer_command;
} basicmicro_ros2__srv__MoveToAbsolutePosition_Request;

// Struct for a sequence of basicmicro_ros2__srv__MoveToAbsolutePosition_Request.
typedef struct basicmicro_ros2__srv__MoveToAbsolutePosition_Request__Sequence
{
  basicmicro_ros2__srv__MoveToAbsolutePosition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__MoveToAbsolutePosition_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveToAbsolutePosition in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__MoveToAbsolutePosition_Response
{
  /// Response
  /// True if command was executed successfully
  bool success;
  /// Status message or error description
  rosidl_runtime_c__String message;
} basicmicro_ros2__srv__MoveToAbsolutePosition_Response;

// Struct for a sequence of basicmicro_ros2__srv__MoveToAbsolutePosition_Response.
typedef struct basicmicro_ros2__srv__MoveToAbsolutePosition_Response__Sequence
{
  basicmicro_ros2__srv__MoveToAbsolutePosition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__MoveToAbsolutePosition_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  basicmicro_ros2__srv__MoveToAbsolutePosition_Event__request__MAX_SIZE = 1
};
// response
enum
{
  basicmicro_ros2__srv__MoveToAbsolutePosition_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/MoveToAbsolutePosition in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__MoveToAbsolutePosition_Event
{
  service_msgs__msg__ServiceEventInfo info;
  basicmicro_ros2__srv__MoveToAbsolutePosition_Request__Sequence request;
  basicmicro_ros2__srv__MoveToAbsolutePosition_Response__Sequence response;
} basicmicro_ros2__srv__MoveToAbsolutePosition_Event;

// Struct for a sequence of basicmicro_ros2__srv__MoveToAbsolutePosition_Event.
typedef struct basicmicro_ros2__srv__MoveToAbsolutePosition_Event__Sequence
{
  basicmicro_ros2__srv__MoveToAbsolutePosition_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__MoveToAbsolutePosition_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__SRV__DETAIL__MOVE_TO_ABSOLUTE_POSITION__STRUCT_H_
