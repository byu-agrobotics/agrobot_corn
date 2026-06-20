// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:srv/ReleasePositionHold.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/release_position_hold.h"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__RELEASE_POSITION_HOLD__STRUCT_H_
#define BASICMICRO_ROS2__SRV__DETAIL__RELEASE_POSITION_HOLD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ReleasePositionHold in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__ReleasePositionHold_Request
{
  uint8_t structure_needs_at_least_one_member;
} basicmicro_ros2__srv__ReleasePositionHold_Request;

// Struct for a sequence of basicmicro_ros2__srv__ReleasePositionHold_Request.
typedef struct basicmicro_ros2__srv__ReleasePositionHold_Request__Sequence
{
  basicmicro_ros2__srv__ReleasePositionHold_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__ReleasePositionHold_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ReleasePositionHold in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__ReleasePositionHold_Response
{
  /// Response
  /// True if position hold was released successfully
  bool success;
  /// Status message or error description
  rosidl_runtime_c__String message;
} basicmicro_ros2__srv__ReleasePositionHold_Response;

// Struct for a sequence of basicmicro_ros2__srv__ReleasePositionHold_Response.
typedef struct basicmicro_ros2__srv__ReleasePositionHold_Response__Sequence
{
  basicmicro_ros2__srv__ReleasePositionHold_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__ReleasePositionHold_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  basicmicro_ros2__srv__ReleasePositionHold_Event__request__MAX_SIZE = 1
};
// response
enum
{
  basicmicro_ros2__srv__ReleasePositionHold_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ReleasePositionHold in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__ReleasePositionHold_Event
{
  service_msgs__msg__ServiceEventInfo info;
  basicmicro_ros2__srv__ReleasePositionHold_Request__Sequence request;
  basicmicro_ros2__srv__ReleasePositionHold_Response__Sequence response;
} basicmicro_ros2__srv__ReleasePositionHold_Event;

// Struct for a sequence of basicmicro_ros2__srv__ReleasePositionHold_Event.
typedef struct basicmicro_ros2__srv__ReleasePositionHold_Event__Sequence
{
  basicmicro_ros2__srv__ReleasePositionHold_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__ReleasePositionHold_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__SRV__DETAIL__RELEASE_POSITION_HOLD__STRUCT_H_
