// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:srv/GetPositionLimits.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/get_position_limits.h"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__GET_POSITION_LIMITS__STRUCT_H_
#define BASICMICRO_ROS2__SRV__DETAIL__GET_POSITION_LIMITS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetPositionLimits in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__GetPositionLimits_Request
{
  uint8_t structure_needs_at_least_one_member;
} basicmicro_ros2__srv__GetPositionLimits_Request;

// Struct for a sequence of basicmicro_ros2__srv__GetPositionLimits_Request.
typedef struct basicmicro_ros2__srv__GetPositionLimits_Request__Sequence
{
  basicmicro_ros2__srv__GetPositionLimits_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__GetPositionLimits_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'violation_behavior'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetPositionLimits in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__GetPositionLimits_Response
{
  /// Response: Current position limit configuration
  /// Whether position limits are currently enabled
  bool limits_enabled;
  /// Current minimum position for left wheel (radians)
  double left_min_position;
  /// Current maximum position for left wheel (radians)
  double left_max_position;
  /// Current minimum position for right wheel (radians)
  double right_min_position;
  /// Current maximum position for right wheel (radians)
  double right_max_position;
  /// Current violation handling: 'hard_stop', 'soft_stop', 'warning'
  rosidl_runtime_c__String violation_behavior;
  /// Current deceleration rate for soft stops (m/s²)
  double decel_rate;
} basicmicro_ros2__srv__GetPositionLimits_Response;

// Struct for a sequence of basicmicro_ros2__srv__GetPositionLimits_Response.
typedef struct basicmicro_ros2__srv__GetPositionLimits_Response__Sequence
{
  basicmicro_ros2__srv__GetPositionLimits_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__GetPositionLimits_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  basicmicro_ros2__srv__GetPositionLimits_Event__request__MAX_SIZE = 1
};
// response
enum
{
  basicmicro_ros2__srv__GetPositionLimits_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetPositionLimits in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__GetPositionLimits_Event
{
  service_msgs__msg__ServiceEventInfo info;
  basicmicro_ros2__srv__GetPositionLimits_Request__Sequence request;
  basicmicro_ros2__srv__GetPositionLimits_Response__Sequence response;
} basicmicro_ros2__srv__GetPositionLimits_Event;

// Struct for a sequence of basicmicro_ros2__srv__GetPositionLimits_Event.
typedef struct basicmicro_ros2__srv__GetPositionLimits_Event__Sequence
{
  basicmicro_ros2__srv__GetPositionLimits_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__GetPositionLimits_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__SRV__DETAIL__GET_POSITION_LIMITS__STRUCT_H_
