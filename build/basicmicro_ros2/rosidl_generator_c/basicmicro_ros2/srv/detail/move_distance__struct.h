// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:srv/MoveDistance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/move_distance.h"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__MOVE_DISTANCE__STRUCT_H_
#define BASICMICRO_ROS2__SRV__DETAIL__MOVE_DISTANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/MoveDistance in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__MoveDistance_Request
{
  /// Distance to travel for left wheel (meters)
  double left_distance;
  /// Distance to travel for right wheel (meters)
  double right_distance;
  /// Target speed during movement (m/s)
  double speed;
  /// Acceleration rate (m/s²)
  double acceleration;
  /// Whether to buffer the command for chaining
  bool use_buffer;
} basicmicro_ros2__srv__MoveDistance_Request;

// Struct for a sequence of basicmicro_ros2__srv__MoveDistance_Request.
typedef struct basicmicro_ros2__srv__MoveDistance_Request__Sequence
{
  basicmicro_ros2__srv__MoveDistance_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__MoveDistance_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveDistance in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__MoveDistance_Response
{
  /// Response: Command execution result
  /// True if command was accepted and executed
  bool success;
  /// Detailed result message or error description
  rosidl_runtime_c__String message;
  /// Number of buffer slots used after this command
  int32_t buffer_slots_used;
} basicmicro_ros2__srv__MoveDistance_Response;

// Struct for a sequence of basicmicro_ros2__srv__MoveDistance_Response.
typedef struct basicmicro_ros2__srv__MoveDistance_Response__Sequence
{
  basicmicro_ros2__srv__MoveDistance_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__MoveDistance_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  basicmicro_ros2__srv__MoveDistance_Event__request__MAX_SIZE = 1
};
// response
enum
{
  basicmicro_ros2__srv__MoveDistance_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/MoveDistance in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__MoveDistance_Event
{
  service_msgs__msg__ServiceEventInfo info;
  basicmicro_ros2__srv__MoveDistance_Request__Sequence request;
  basicmicro_ros2__srv__MoveDistance_Response__Sequence response;
} basicmicro_ros2__srv__MoveDistance_Event;

// Struct for a sequence of basicmicro_ros2__srv__MoveDistance_Event.
typedef struct basicmicro_ros2__srv__MoveDistance_Event__Sequence
{
  basicmicro_ros2__srv__MoveDistance_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__MoveDistance_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__SRV__DETAIL__MOVE_DISTANCE__STRUCT_H_
