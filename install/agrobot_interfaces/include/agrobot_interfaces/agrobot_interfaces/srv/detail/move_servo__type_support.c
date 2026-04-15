// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from agrobot_interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "agrobot_interfaces/srv/detail/move_servo__rosidl_typesupport_introspection_c.h"
#include "agrobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "agrobot_interfaces/srv/detail/move_servo__functions.h"
#include "agrobot_interfaces/srv/detail/move_servo__struct.h"


// Include directives for member types
// Member `request`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  agrobot_interfaces__srv__MoveServo_Request__init(message_memory);
}

void agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_fini_function(void * message_memory)
{
  agrobot_interfaces__srv__MoveServo_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_message_member_array[1] = {
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(agrobot_interfaces__srv__MoveServo_Request, request),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_message_members = {
  "agrobot_interfaces__srv",  // message namespace
  "MoveServo_Request",  // message name
  1,  // number of fields
  sizeof(agrobot_interfaces__srv__MoveServo_Request),
  false,  // has_any_key_member_
  agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_message_member_array,  // message members
  agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_message_type_support_handle = {
  0,
  &agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_message_members,
  get_message_typesupport_handle_function,
  &agrobot_interfaces__srv__MoveServo_Request__get_type_hash,
  &agrobot_interfaces__srv__MoveServo_Request__get_type_description,
  &agrobot_interfaces__srv__MoveServo_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_agrobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Request)() {
  if (!agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_message_type_support_handle.typesupport_identifier) {
    agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "agrobot_interfaces/srv/detail/move_servo__rosidl_typesupport_introspection_c.h"
// already included above
// #include "agrobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "agrobot_interfaces/srv/detail/move_servo__functions.h"
// already included above
// #include "agrobot_interfaces/srv/detail/move_servo__struct.h"


// Include directives for member types
// Member `response`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  agrobot_interfaces__srv__MoveServo_Response__init(message_memory);
}

void agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_fini_function(void * message_memory)
{
  agrobot_interfaces__srv__MoveServo_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_message_member_array[1] = {
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(agrobot_interfaces__srv__MoveServo_Response, response),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_message_members = {
  "agrobot_interfaces__srv",  // message namespace
  "MoveServo_Response",  // message name
  1,  // number of fields
  sizeof(agrobot_interfaces__srv__MoveServo_Response),
  false,  // has_any_key_member_
  agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_message_member_array,  // message members
  agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_message_type_support_handle = {
  0,
  &agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_message_members,
  get_message_typesupport_handle_function,
  &agrobot_interfaces__srv__MoveServo_Response__get_type_hash,
  &agrobot_interfaces__srv__MoveServo_Response__get_type_description,
  &agrobot_interfaces__srv__MoveServo_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_agrobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Response)() {
  if (!agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_message_type_support_handle.typesupport_identifier) {
    agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "agrobot_interfaces/srv/detail/move_servo__rosidl_typesupport_introspection_c.h"
// already included above
// #include "agrobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "agrobot_interfaces/srv/detail/move_servo__functions.h"
// already included above
// #include "agrobot_interfaces/srv/detail/move_servo__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "agrobot_interfaces/srv/move_servo.h"
// Member `request`
// Member `response`
// already included above
// #include "agrobot_interfaces/srv/detail/move_servo__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  agrobot_interfaces__srv__MoveServo_Event__init(message_memory);
}

void agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_fini_function(void * message_memory)
{
  agrobot_interfaces__srv__MoveServo_Event__fini(message_memory);
}

size_t agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__size_function__MoveServo_Event__request(
  const void * untyped_member)
{
  const agrobot_interfaces__srv__MoveServo_Request__Sequence * member =
    (const agrobot_interfaces__srv__MoveServo_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_const_function__MoveServo_Event__request(
  const void * untyped_member, size_t index)
{
  const agrobot_interfaces__srv__MoveServo_Request__Sequence * member =
    (const agrobot_interfaces__srv__MoveServo_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_function__MoveServo_Event__request(
  void * untyped_member, size_t index)
{
  agrobot_interfaces__srv__MoveServo_Request__Sequence * member =
    (agrobot_interfaces__srv__MoveServo_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__fetch_function__MoveServo_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const agrobot_interfaces__srv__MoveServo_Request * item =
    ((const agrobot_interfaces__srv__MoveServo_Request *)
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_const_function__MoveServo_Event__request(untyped_member, index));
  agrobot_interfaces__srv__MoveServo_Request * value =
    (agrobot_interfaces__srv__MoveServo_Request *)(untyped_value);
  *value = *item;
}

void agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__assign_function__MoveServo_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  agrobot_interfaces__srv__MoveServo_Request * item =
    ((agrobot_interfaces__srv__MoveServo_Request *)
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_function__MoveServo_Event__request(untyped_member, index));
  const agrobot_interfaces__srv__MoveServo_Request * value =
    (const agrobot_interfaces__srv__MoveServo_Request *)(untyped_value);
  *item = *value;
}

bool agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__resize_function__MoveServo_Event__request(
  void * untyped_member, size_t size)
{
  agrobot_interfaces__srv__MoveServo_Request__Sequence * member =
    (agrobot_interfaces__srv__MoveServo_Request__Sequence *)(untyped_member);
  agrobot_interfaces__srv__MoveServo_Request__Sequence__fini(member);
  return agrobot_interfaces__srv__MoveServo_Request__Sequence__init(member, size);
}

size_t agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__size_function__MoveServo_Event__response(
  const void * untyped_member)
{
  const agrobot_interfaces__srv__MoveServo_Response__Sequence * member =
    (const agrobot_interfaces__srv__MoveServo_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_const_function__MoveServo_Event__response(
  const void * untyped_member, size_t index)
{
  const agrobot_interfaces__srv__MoveServo_Response__Sequence * member =
    (const agrobot_interfaces__srv__MoveServo_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_function__MoveServo_Event__response(
  void * untyped_member, size_t index)
{
  agrobot_interfaces__srv__MoveServo_Response__Sequence * member =
    (agrobot_interfaces__srv__MoveServo_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__fetch_function__MoveServo_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const agrobot_interfaces__srv__MoveServo_Response * item =
    ((const agrobot_interfaces__srv__MoveServo_Response *)
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_const_function__MoveServo_Event__response(untyped_member, index));
  agrobot_interfaces__srv__MoveServo_Response * value =
    (agrobot_interfaces__srv__MoveServo_Response *)(untyped_value);
  *value = *item;
}

void agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__assign_function__MoveServo_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  agrobot_interfaces__srv__MoveServo_Response * item =
    ((agrobot_interfaces__srv__MoveServo_Response *)
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_function__MoveServo_Event__response(untyped_member, index));
  const agrobot_interfaces__srv__MoveServo_Response * value =
    (const agrobot_interfaces__srv__MoveServo_Response *)(untyped_value);
  *item = *value;
}

bool agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__resize_function__MoveServo_Event__response(
  void * untyped_member, size_t size)
{
  agrobot_interfaces__srv__MoveServo_Response__Sequence * member =
    (agrobot_interfaces__srv__MoveServo_Response__Sequence *)(untyped_member);
  agrobot_interfaces__srv__MoveServo_Response__Sequence__fini(member);
  return agrobot_interfaces__srv__MoveServo_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(agrobot_interfaces__srv__MoveServo_Event, info),  // bytes offset in struct
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
    offsetof(agrobot_interfaces__srv__MoveServo_Event, request),  // bytes offset in struct
    NULL,  // default value
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__size_function__MoveServo_Event__request,  // size() function pointer
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_const_function__MoveServo_Event__request,  // get_const(index) function pointer
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_function__MoveServo_Event__request,  // get(index) function pointer
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__fetch_function__MoveServo_Event__request,  // fetch(index, &value) function pointer
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__assign_function__MoveServo_Event__request,  // assign(index, value) function pointer
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__resize_function__MoveServo_Event__request  // resize(index) function pointer
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
    offsetof(agrobot_interfaces__srv__MoveServo_Event, response),  // bytes offset in struct
    NULL,  // default value
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__size_function__MoveServo_Event__response,  // size() function pointer
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_const_function__MoveServo_Event__response,  // get_const(index) function pointer
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__get_function__MoveServo_Event__response,  // get(index) function pointer
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__fetch_function__MoveServo_Event__response,  // fetch(index, &value) function pointer
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__assign_function__MoveServo_Event__response,  // assign(index, value) function pointer
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__resize_function__MoveServo_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_members = {
  "agrobot_interfaces__srv",  // message namespace
  "MoveServo_Event",  // message name
  3,  // number of fields
  sizeof(agrobot_interfaces__srv__MoveServo_Event),
  false,  // has_any_key_member_
  agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_member_array,  // message members
  agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_type_support_handle = {
  0,
  &agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_members,
  get_message_typesupport_handle_function,
  &agrobot_interfaces__srv__MoveServo_Event__get_type_hash,
  &agrobot_interfaces__srv__MoveServo_Event__get_type_description,
  &agrobot_interfaces__srv__MoveServo_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_agrobot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Event)() {
  agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Request)();
  agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Response)();
  if (!agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_type_support_handle.typesupport_identifier) {
    agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "agrobot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "agrobot_interfaces/srv/detail/move_servo__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers agrobot_interfaces__srv__detail__move_servo__rosidl_typesupport_introspection_c__MoveServo_service_members = {
  "agrobot_interfaces__srv",  // service namespace
  "MoveServo",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // agrobot_interfaces__srv__detail__move_servo__rosidl_typesupport_introspection_c__MoveServo_Request_message_type_support_handle,
  NULL,  // response message
  // agrobot_interfaces__srv__detail__move_servo__rosidl_typesupport_introspection_c__MoveServo_Response_message_type_support_handle
  NULL  // event_message
  // agrobot_interfaces__srv__detail__move_servo__rosidl_typesupport_introspection_c__MoveServo_Response_message_type_support_handle
};


static rosidl_service_type_support_t agrobot_interfaces__srv__detail__move_servo__rosidl_typesupport_introspection_c__MoveServo_service_type_support_handle = {
  0,
  &agrobot_interfaces__srv__detail__move_servo__rosidl_typesupport_introspection_c__MoveServo_service_members,
  get_service_typesupport_handle_function,
  &agrobot_interfaces__srv__MoveServo_Request__rosidl_typesupport_introspection_c__MoveServo_Request_message_type_support_handle,
  &agrobot_interfaces__srv__MoveServo_Response__rosidl_typesupport_introspection_c__MoveServo_Response_message_type_support_handle,
  &agrobot_interfaces__srv__MoveServo_Event__rosidl_typesupport_introspection_c__MoveServo_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    agrobot_interfaces,
    srv,
    MoveServo
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    agrobot_interfaces,
    srv,
    MoveServo
  ),
  &agrobot_interfaces__srv__MoveServo__get_type_hash,
  &agrobot_interfaces__srv__MoveServo__get_type_description,
  &agrobot_interfaces__srv__MoveServo__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_agrobot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo)(void) {
  if (!agrobot_interfaces__srv__detail__move_servo__rosidl_typesupport_introspection_c__MoveServo_service_type_support_handle.typesupport_identifier) {
    agrobot_interfaces__srv__detail__move_servo__rosidl_typesupport_introspection_c__MoveServo_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)agrobot_interfaces__srv__detail__move_servo__rosidl_typesupport_introspection_c__MoveServo_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agrobot_interfaces, srv, MoveServo_Event)()->data;
  }

  return &agrobot_interfaces__srv__detail__move_servo__rosidl_typesupport_introspection_c__MoveServo_service_type_support_handle;
}
