// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:srv/ExecutePositionSequence.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/execute_position_sequence.h"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_POSITION_SEQUENCE__STRUCT_H_
#define BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_POSITION_SEQUENCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position_points'
#include "basicmicro_ros2/msg/detail/position_point__struct.h"

/// Struct defined in srv/ExecutePositionSequence in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__ExecutePositionSequence_Request
{
  /// Position points to execute in sequence
  basicmicro_ros2__msg__PositionPoint__Sequence position_points;
} basicmicro_ros2__srv__ExecutePositionSequence_Request;

// Struct for a sequence of basicmicro_ros2__srv__ExecutePositionSequence_Request.
typedef struct basicmicro_ros2__srv__ExecutePositionSequence_Request__Sequence
{
  basicmicro_ros2__srv__ExecutePositionSequence_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__ExecutePositionSequence_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ExecutePositionSequence in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__ExecutePositionSequence_Response
{
  /// True if sequence was successfully queued
  bool success;
  /// Status message or error description
  rosidl_runtime_c__String message;
  /// Number of position commands successfully sent to buffer
  int32_t total_commands_sent;
} basicmicro_ros2__srv__ExecutePositionSequence_Response;

// Struct for a sequence of basicmicro_ros2__srv__ExecutePositionSequence_Response.
typedef struct basicmicro_ros2__srv__ExecutePositionSequence_Response__Sequence
{
  basicmicro_ros2__srv__ExecutePositionSequence_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__ExecutePositionSequence_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  basicmicro_ros2__srv__ExecutePositionSequence_Event__request__MAX_SIZE = 1
};
// response
enum
{
  basicmicro_ros2__srv__ExecutePositionSequence_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ExecutePositionSequence in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__ExecutePositionSequence_Event
{
  service_msgs__msg__ServiceEventInfo info;
  basicmicro_ros2__srv__ExecutePositionSequence_Request__Sequence request;
  basicmicro_ros2__srv__ExecutePositionSequence_Response__Sequence response;
} basicmicro_ros2__srv__ExecutePositionSequence_Event;

// Struct for a sequence of basicmicro_ros2__srv__ExecutePositionSequence_Event.
typedef struct basicmicro_ros2__srv__ExecutePositionSequence_Event__Sequence
{
  basicmicro_ros2__srv__ExecutePositionSequence_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__ExecutePositionSequence_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_POSITION_SEQUENCE__STRUCT_H_
