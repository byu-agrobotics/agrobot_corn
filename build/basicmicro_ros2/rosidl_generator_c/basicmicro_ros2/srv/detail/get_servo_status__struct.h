// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:srv/GetServoStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/get_servo_status.h"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__GET_SERVO_STATUS__STRUCT_H_
#define BASICMICRO_ROS2__SRV__DETAIL__GET_SERVO_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetServoStatus in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__GetServoStatus_Request
{
  uint8_t structure_needs_at_least_one_member;
} basicmicro_ros2__srv__GetServoStatus_Request;

// Struct for a sequence of basicmicro_ros2__srv__GetServoStatus_Request.
typedef struct basicmicro_ros2__srv__GetServoStatus_Request__Sequence
{
  basicmicro_ros2__srv__GetServoStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__GetServoStatus_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetServoStatus in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__GetServoStatus_Response
{
  /// Response
  /// True if status was read successfully
  bool success;
  /// Current position error for left motor (encoder counts)
  int32_t left_position_error;
  /// Current position error for right motor (encoder counts)
  int32_t right_position_error;
  /// Current speed error for left motor (counts/sec)
  int32_t left_speed_error;
  /// Current speed error for right motor (counts/sec)
  int32_t right_speed_error;
  /// Whether controller is in error state (requires POR)
  bool error_limits_exceeded;
  /// Status message or error description
  rosidl_runtime_c__String message;
} basicmicro_ros2__srv__GetServoStatus_Response;

// Struct for a sequence of basicmicro_ros2__srv__GetServoStatus_Response.
typedef struct basicmicro_ros2__srv__GetServoStatus_Response__Sequence
{
  basicmicro_ros2__srv__GetServoStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__GetServoStatus_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  basicmicro_ros2__srv__GetServoStatus_Event__request__MAX_SIZE = 1
};
// response
enum
{
  basicmicro_ros2__srv__GetServoStatus_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetServoStatus in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__GetServoStatus_Event
{
  service_msgs__msg__ServiceEventInfo info;
  basicmicro_ros2__srv__GetServoStatus_Request__Sequence request;
  basicmicro_ros2__srv__GetServoStatus_Response__Sequence response;
} basicmicro_ros2__srv__GetServoStatus_Event;

// Struct for a sequence of basicmicro_ros2__srv__GetServoStatus_Event.
typedef struct basicmicro_ros2__srv__GetServoStatus_Event__Sequence
{
  basicmicro_ros2__srv__GetServoStatus_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__GetServoStatus_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__SRV__DETAIL__GET_SERVO_STATUS__STRUCT_H_
