// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:srv/GetAvailableHomingMethods.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/get_available_homing_methods.h"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__GET_AVAILABLE_HOMING_METHODS__STRUCT_H_
#define BASICMICRO_ROS2__SRV__DETAIL__GET_AVAILABLE_HOMING_METHODS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetAvailableHomingMethods in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__GetAvailableHomingMethods_Request
{
  uint8_t structure_needs_at_least_one_member;
} basicmicro_ros2__srv__GetAvailableHomingMethods_Request;

// Struct for a sequence of basicmicro_ros2__srv__GetAvailableHomingMethods_Request.
typedef struct basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'controller_type'
// Member 'available_methods'
// Member 'method_descriptions'
// Member 'allowed_directions'
#include "rosidl_runtime_c/string.h"
// Member 'auto_zeros_encoder'
// Member 'acts_as_limit'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/GetAvailableHomingMethods in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__GetAvailableHomingMethods_Response
{
  /// Response
  /// True if query was successful
  bool success;
  /// "roboclaw" or "mcp"
  rosidl_runtime_c__String controller_type;
  /// List of available homing method IDs
  rosidl_runtime_c__String__Sequence available_methods;
  /// Human-readable descriptions for each method
  rosidl_runtime_c__String__Sequence method_descriptions;
  /// Allowed directions for each method (forward/backward/both)
  rosidl_runtime_c__String__Sequence allowed_directions;
  /// Whether each method automatically zeros encoder
  rosidl_runtime_c__boolean__Sequence auto_zeros_encoder;
  /// Whether each method also acts as limit switch
  rosidl_runtime_c__boolean__Sequence acts_as_limit;
} basicmicro_ros2__srv__GetAvailableHomingMethods_Response;

// Struct for a sequence of basicmicro_ros2__srv__GetAvailableHomingMethods_Response.
typedef struct basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event__request__MAX_SIZE = 1
};
// response
enum
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetAvailableHomingMethods in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__GetAvailableHomingMethods_Event
{
  service_msgs__msg__ServiceEventInfo info;
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence request;
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence response;
} basicmicro_ros2__srv__GetAvailableHomingMethods_Event;

// Struct for a sequence of basicmicro_ros2__srv__GetAvailableHomingMethods_Event.
typedef struct basicmicro_ros2__srv__GetAvailableHomingMethods_Event__Sequence
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__GetAvailableHomingMethods_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__SRV__DETAIL__GET_AVAILABLE_HOMING_METHODS__STRUCT_H_
