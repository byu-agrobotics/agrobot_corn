// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from agrobot_interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "agrobot_interfaces/srv/move_servo.h"


#ifndef AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_H_
#define AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'request'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveServo in the package agrobot_interfaces.
typedef struct agrobot_interfaces__srv__MoveServo_Request
{
  rosidl_runtime_c__String request;
} agrobot_interfaces__srv__MoveServo_Request;

// Struct for a sequence of agrobot_interfaces__srv__MoveServo_Request.
typedef struct agrobot_interfaces__srv__MoveServo_Request__Sequence
{
  agrobot_interfaces__srv__MoveServo_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} agrobot_interfaces__srv__MoveServo_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'response'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveServo in the package agrobot_interfaces.
typedef struct agrobot_interfaces__srv__MoveServo_Response
{
  rosidl_runtime_c__String response;
} agrobot_interfaces__srv__MoveServo_Response;

// Struct for a sequence of agrobot_interfaces__srv__MoveServo_Response.
typedef struct agrobot_interfaces__srv__MoveServo_Response__Sequence
{
  agrobot_interfaces__srv__MoveServo_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} agrobot_interfaces__srv__MoveServo_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  agrobot_interfaces__srv__MoveServo_Event__request__MAX_SIZE = 1
};
// response
enum
{
  agrobot_interfaces__srv__MoveServo_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/MoveServo in the package agrobot_interfaces.
typedef struct agrobot_interfaces__srv__MoveServo_Event
{
  service_msgs__msg__ServiceEventInfo info;
  agrobot_interfaces__srv__MoveServo_Request__Sequence request;
  agrobot_interfaces__srv__MoveServo_Response__Sequence response;
} agrobot_interfaces__srv__MoveServo_Event;

// Struct for a sequence of agrobot_interfaces__srv__MoveServo_Event.
typedef struct agrobot_interfaces__srv__MoveServo_Event__Sequence
{
  agrobot_interfaces__srv__MoveServo_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} agrobot_interfaces__srv__MoveServo_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_H_
