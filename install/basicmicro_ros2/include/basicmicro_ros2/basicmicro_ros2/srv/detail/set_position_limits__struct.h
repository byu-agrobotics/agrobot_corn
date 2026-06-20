// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:srv/SetPositionLimits.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/set_position_limits.h"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__SET_POSITION_LIMITS__STRUCT_H_
#define BASICMICRO_ROS2__SRV__DETAIL__SET_POSITION_LIMITS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'violation_behavior'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetPositionLimits in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__SetPositionLimits_Request
{
  /// Enable or disable position limit checking
  bool enable_limits;
  /// Minimum position for left wheel (radians)
  double left_min_position;
  /// Maximum position for left wheel (radians)
  double left_max_position;
  /// Minimum position for right wheel (radians)
  double right_min_position;
  /// Maximum position for right wheel (radians)
  double right_max_position;
  /// How to handle limit violations: 'hard_stop', 'soft_stop', 'warning'
  rosidl_runtime_c__String violation_behavior;
  /// Deceleration rate for soft stops (m/s²)
  double decel_rate;
} basicmicro_ros2__srv__SetPositionLimits_Request;

// Struct for a sequence of basicmicro_ros2__srv__SetPositionLimits_Request.
typedef struct basicmicro_ros2__srv__SetPositionLimits_Request__Sequence
{
  basicmicro_ros2__srv__SetPositionLimits_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__SetPositionLimits_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetPositionLimits in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__SetPositionLimits_Response
{
  /// Response: Configuration result
  /// True if limits were successfully configured
  bool success;
  /// Detailed result message or error description
  rosidl_runtime_c__String message;
} basicmicro_ros2__srv__SetPositionLimits_Response;

// Struct for a sequence of basicmicro_ros2__srv__SetPositionLimits_Response.
typedef struct basicmicro_ros2__srv__SetPositionLimits_Response__Sequence
{
  basicmicro_ros2__srv__SetPositionLimits_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__SetPositionLimits_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  basicmicro_ros2__srv__SetPositionLimits_Event__request__MAX_SIZE = 1
};
// response
enum
{
  basicmicro_ros2__srv__SetPositionLimits_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetPositionLimits in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__SetPositionLimits_Event
{
  service_msgs__msg__ServiceEventInfo info;
  basicmicro_ros2__srv__SetPositionLimits_Request__Sequence request;
  basicmicro_ros2__srv__SetPositionLimits_Response__Sequence response;
} basicmicro_ros2__srv__SetPositionLimits_Event;

// Struct for a sequence of basicmicro_ros2__srv__SetPositionLimits_Event.
typedef struct basicmicro_ros2__srv__SetPositionLimits_Event__Sequence
{
  basicmicro_ros2__srv__SetPositionLimits_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__SetPositionLimits_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__SRV__DETAIL__SET_POSITION_LIMITS__STRUCT_H_
