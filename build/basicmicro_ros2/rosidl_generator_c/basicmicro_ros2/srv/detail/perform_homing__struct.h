// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:srv/PerformHoming.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/perform_homing.h"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__PERFORM_HOMING__STRUCT_H_
#define BASICMICRO_ROS2__SRV__DETAIL__PERFORM_HOMING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'method_id'
// Member 'direction'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PerformHoming in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__PerformHoming_Request
{
  /// Request
  /// Method ID from available methods
  rosidl_runtime_c__String method_id;
  /// "forward", "backward" (if method supports choice)
  rosidl_runtime_c__String direction;
  /// Homing speed (m/s)
  double homing_speed;
  /// Timeout for homing operation (seconds)
  double timeout;
} basicmicro_ros2__srv__PerformHoming_Request;

// Struct for a sequence of basicmicro_ros2__srv__PerformHoming_Request.
typedef struct basicmicro_ros2__srv__PerformHoming_Request__Sequence
{
  basicmicro_ros2__srv__PerformHoming_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__PerformHoming_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PerformHoming in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__PerformHoming_Response
{
  /// Response
  /// True if homing was successful
  bool success;
  /// Status message or error description
  rosidl_runtime_c__String message;
  /// Whether encoder was automatically zeroed
  bool encoder_zeroed;
} basicmicro_ros2__srv__PerformHoming_Response;

// Struct for a sequence of basicmicro_ros2__srv__PerformHoming_Response.
typedef struct basicmicro_ros2__srv__PerformHoming_Response__Sequence
{
  basicmicro_ros2__srv__PerformHoming_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__PerformHoming_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  basicmicro_ros2__srv__PerformHoming_Event__request__MAX_SIZE = 1
};
// response
enum
{
  basicmicro_ros2__srv__PerformHoming_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/PerformHoming in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__PerformHoming_Event
{
  service_msgs__msg__ServiceEventInfo info;
  basicmicro_ros2__srv__PerformHoming_Request__Sequence request;
  basicmicro_ros2__srv__PerformHoming_Response__Sequence response;
} basicmicro_ros2__srv__PerformHoming_Event;

// Struct for a sequence of basicmicro_ros2__srv__PerformHoming_Event.
typedef struct basicmicro_ros2__srv__PerformHoming_Event__Sequence
{
  basicmicro_ros2__srv__PerformHoming_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__PerformHoming_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__SRV__DETAIL__PERFORM_HOMING__STRUCT_H_
