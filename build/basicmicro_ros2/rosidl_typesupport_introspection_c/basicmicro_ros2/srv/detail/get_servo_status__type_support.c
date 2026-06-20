// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from basicmicro_ros2:srv/GetServoStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "basicmicro_ros2/srv/detail/get_servo_status__rosidl_typesupport_introspection_c.h"
#include "basicmicro_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "basicmicro_ros2/srv/detail/get_servo_status__functions.h"
#include "basicmicro_ros2/srv/detail/get_servo_status__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  basicmicro_ros2__srv__GetServoStatus_Request__init(message_memory);
}

void basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_fini_function(void * message_memory)
{
  basicmicro_ros2__srv__GetServoStatus_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_message_members = {
  "basicmicro_ros2__srv",  // message namespace
  "GetServoStatus_Request",  // message name
  1,  // number of fields
  sizeof(basicmicro_ros2__srv__GetServoStatus_Request),
  false,  // has_any_key_member_
  basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_message_member_array,  // message members
  basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_message_type_support_handle = {
  0,
  &basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetServoStatus_Request__get_type_hash,
  &basicmicro_ros2__srv__GetServoStatus_Request__get_type_description,
  &basicmicro_ros2__srv__GetServoStatus_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_basicmicro_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Request)() {
  if (!basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_message_type_support_handle.typesupport_identifier) {
    basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__rosidl_typesupport_introspection_c.h"
// already included above
// #include "basicmicro_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  basicmicro_ros2__srv__GetServoStatus_Response__init(message_memory);
}

void basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_fini_function(void * message_memory)
{
  basicmicro_ros2__srv__GetServoStatus_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_member_array[7] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_position_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Response, left_position_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_position_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Response, right_position_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_speed_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Response, left_speed_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_speed_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Response, right_speed_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_limits_exceeded",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Response, error_limits_exceeded),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_members = {
  "basicmicro_ros2__srv",  // message namespace
  "GetServoStatus_Response",  // message name
  7,  // number of fields
  sizeof(basicmicro_ros2__srv__GetServoStatus_Response),
  false,  // has_any_key_member_
  basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_member_array,  // message members
  basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_type_support_handle = {
  0,
  &basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetServoStatus_Response__get_type_hash,
  &basicmicro_ros2__srv__GetServoStatus_Response__get_type_description,
  &basicmicro_ros2__srv__GetServoStatus_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_basicmicro_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Response)() {
  if (!basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_type_support_handle.typesupport_identifier) {
    basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__rosidl_typesupport_introspection_c.h"
// already included above
// #include "basicmicro_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "basicmicro_ros2/srv/get_servo_status.h"
// Member `request`
// Member `response`
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  basicmicro_ros2__srv__GetServoStatus_Event__init(message_memory);
}

void basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_fini_function(void * message_memory)
{
  basicmicro_ros2__srv__GetServoStatus_Event__fini(message_memory);
}

size_t basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__size_function__GetServoStatus_Event__request(
  const void * untyped_member)
{
  const basicmicro_ros2__srv__GetServoStatus_Request__Sequence * member =
    (const basicmicro_ros2__srv__GetServoStatus_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetServoStatus_Event__request(
  const void * untyped_member, size_t index)
{
  const basicmicro_ros2__srv__GetServoStatus_Request__Sequence * member =
    (const basicmicro_ros2__srv__GetServoStatus_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_function__GetServoStatus_Event__request(
  void * untyped_member, size_t index)
{
  basicmicro_ros2__srv__GetServoStatus_Request__Sequence * member =
    (basicmicro_ros2__srv__GetServoStatus_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__fetch_function__GetServoStatus_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const basicmicro_ros2__srv__GetServoStatus_Request * item =
    ((const basicmicro_ros2__srv__GetServoStatus_Request *)
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetServoStatus_Event__request(untyped_member, index));
  basicmicro_ros2__srv__GetServoStatus_Request * value =
    (basicmicro_ros2__srv__GetServoStatus_Request *)(untyped_value);
  *value = *item;
}

void basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__assign_function__GetServoStatus_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  basicmicro_ros2__srv__GetServoStatus_Request * item =
    ((basicmicro_ros2__srv__GetServoStatus_Request *)
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_function__GetServoStatus_Event__request(untyped_member, index));
  const basicmicro_ros2__srv__GetServoStatus_Request * value =
    (const basicmicro_ros2__srv__GetServoStatus_Request *)(untyped_value);
  *item = *value;
}

bool basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__resize_function__GetServoStatus_Event__request(
  void * untyped_member, size_t size)
{
  basicmicro_ros2__srv__GetServoStatus_Request__Sequence * member =
    (basicmicro_ros2__srv__GetServoStatus_Request__Sequence *)(untyped_member);
  basicmicro_ros2__srv__GetServoStatus_Request__Sequence__fini(member);
  return basicmicro_ros2__srv__GetServoStatus_Request__Sequence__init(member, size);
}

size_t basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__size_function__GetServoStatus_Event__response(
  const void * untyped_member)
{
  const basicmicro_ros2__srv__GetServoStatus_Response__Sequence * member =
    (const basicmicro_ros2__srv__GetServoStatus_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetServoStatus_Event__response(
  const void * untyped_member, size_t index)
{
  const basicmicro_ros2__srv__GetServoStatus_Response__Sequence * member =
    (const basicmicro_ros2__srv__GetServoStatus_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_function__GetServoStatus_Event__response(
  void * untyped_member, size_t index)
{
  basicmicro_ros2__srv__GetServoStatus_Response__Sequence * member =
    (basicmicro_ros2__srv__GetServoStatus_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__fetch_function__GetServoStatus_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const basicmicro_ros2__srv__GetServoStatus_Response * item =
    ((const basicmicro_ros2__srv__GetServoStatus_Response *)
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetServoStatus_Event__response(untyped_member, index));
  basicmicro_ros2__srv__GetServoStatus_Response * value =
    (basicmicro_ros2__srv__GetServoStatus_Response *)(untyped_value);
  *value = *item;
}

void basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__assign_function__GetServoStatus_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  basicmicro_ros2__srv__GetServoStatus_Response * item =
    ((basicmicro_ros2__srv__GetServoStatus_Response *)
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_function__GetServoStatus_Event__response(untyped_member, index));
  const basicmicro_ros2__srv__GetServoStatus_Response * value =
    (const basicmicro_ros2__srv__GetServoStatus_Response *)(untyped_value);
  *item = *value;
}

bool basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__resize_function__GetServoStatus_Event__response(
  void * untyped_member, size_t size)
{
  basicmicro_ros2__srv__GetServoStatus_Response__Sequence * member =
    (basicmicro_ros2__srv__GetServoStatus_Response__Sequence *)(untyped_member);
  basicmicro_ros2__srv__GetServoStatus_Response__Sequence__fini(member);
  return basicmicro_ros2__srv__GetServoStatus_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Event, request),  // bytes offset in struct
    NULL,  // default value
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__size_function__GetServoStatus_Event__request,  // size() function pointer
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetServoStatus_Event__request,  // get_const(index) function pointer
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_function__GetServoStatus_Event__request,  // get(index) function pointer
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__fetch_function__GetServoStatus_Event__request,  // fetch(index, &value) function pointer
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__assign_function__GetServoStatus_Event__request,  // assign(index, value) function pointer
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__resize_function__GetServoStatus_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetServoStatus_Event, response),  // bytes offset in struct
    NULL,  // default value
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__size_function__GetServoStatus_Event__response,  // size() function pointer
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_const_function__GetServoStatus_Event__response,  // get_const(index) function pointer
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__get_function__GetServoStatus_Event__response,  // get(index) function pointer
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__fetch_function__GetServoStatus_Event__response,  // fetch(index, &value) function pointer
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__assign_function__GetServoStatus_Event__response,  // assign(index, value) function pointer
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__resize_function__GetServoStatus_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_members = {
  "basicmicro_ros2__srv",  // message namespace
  "GetServoStatus_Event",  // message name
  3,  // number of fields
  sizeof(basicmicro_ros2__srv__GetServoStatus_Event),
  false,  // has_any_key_member_
  basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_member_array,  // message members
  basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_type_support_handle = {
  0,
  &basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetServoStatus_Event__get_type_hash,
  &basicmicro_ros2__srv__GetServoStatus_Event__get_type_description,
  &basicmicro_ros2__srv__GetServoStatus_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_basicmicro_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Event)() {
  basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Request)();
  basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Response)();
  if (!basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_type_support_handle.typesupport_identifier) {
    basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers basicmicro_ros2__srv__detail__get_servo_status__rosidl_typesupport_introspection_c__GetServoStatus_service_members = {
  "basicmicro_ros2__srv",  // service namespace
  "GetServoStatus",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // basicmicro_ros2__srv__detail__get_servo_status__rosidl_typesupport_introspection_c__GetServoStatus_Request_message_type_support_handle,
  NULL,  // response message
  // basicmicro_ros2__srv__detail__get_servo_status__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_type_support_handle
  NULL  // event_message
  // basicmicro_ros2__srv__detail__get_servo_status__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_type_support_handle
};


static rosidl_service_type_support_t basicmicro_ros2__srv__detail__get_servo_status__rosidl_typesupport_introspection_c__GetServoStatus_service_type_support_handle = {
  0,
  &basicmicro_ros2__srv__detail__get_servo_status__rosidl_typesupport_introspection_c__GetServoStatus_service_members,
  get_service_typesupport_handle_function,
  &basicmicro_ros2__srv__GetServoStatus_Request__rosidl_typesupport_introspection_c__GetServoStatus_Request_message_type_support_handle,
  &basicmicro_ros2__srv__GetServoStatus_Response__rosidl_typesupport_introspection_c__GetServoStatus_Response_message_type_support_handle,
  &basicmicro_ros2__srv__GetServoStatus_Event__rosidl_typesupport_introspection_c__GetServoStatus_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    GetServoStatus
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    GetServoStatus
  ),
  &basicmicro_ros2__srv__GetServoStatus__get_type_hash,
  &basicmicro_ros2__srv__GetServoStatus__get_type_description,
  &basicmicro_ros2__srv__GetServoStatus__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_basicmicro_ros2
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus)(void) {
  if (!basicmicro_ros2__srv__detail__get_servo_status__rosidl_typesupport_introspection_c__GetServoStatus_service_type_support_handle.typesupport_identifier) {
    basicmicro_ros2__srv__detail__get_servo_status__rosidl_typesupport_introspection_c__GetServoStatus_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)basicmicro_ros2__srv__detail__get_servo_status__rosidl_typesupport_introspection_c__GetServoStatus_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetServoStatus_Event)()->data;
  }

  return &basicmicro_ros2__srv__detail__get_servo_status__rosidl_typesupport_introspection_c__GetServoStatus_service_type_support_handle;
}
