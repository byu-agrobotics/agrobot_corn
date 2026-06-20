// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from basicmicro_ros2:srv/GetAvailableHomingMethods.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "basicmicro_ros2/srv/detail/get_available_homing_methods__rosidl_typesupport_introspection_c.h"
#include "basicmicro_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
#include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request__init(message_memory);
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_fini_function(void * message_memory)
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_message_members = {
  "basicmicro_ros2__srv",  // message namespace
  "GetAvailableHomingMethods_Request",  // message name
  1,  // number of fields
  sizeof(basicmicro_ros2__srv__GetAvailableHomingMethods_Request),
  false,  // has_any_key_member_
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_message_member_array,  // message members
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_message_type_support_handle = {
  0,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_basicmicro_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)() {
  if (!basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_message_type_support_handle.typesupport_identifier) {
    basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__rosidl_typesupport_introspection_c.h"
// already included above
// #include "basicmicro_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.h"


// Include directives for member types
// Member `controller_type`
// Member `available_methods`
// Member `method_descriptions`
// Member `allowed_directions`
#include "rosidl_runtime_c/string_functions.h"
// Member `auto_zeros_encoder`
// Member `acts_as_limit`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response__init(message_memory);
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_fini_function(void * message_memory)
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response__fini(message_memory);
}

size_t basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Response__available_methods(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__available_methods(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__available_methods(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Response__available_methods(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__available_methods(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Response__available_methods(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__available_methods(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Response__available_methods(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Response__method_descriptions(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__method_descriptions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__method_descriptions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Response__method_descriptions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__method_descriptions(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Response__method_descriptions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__method_descriptions(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Response__method_descriptions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Response__allowed_directions(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__allowed_directions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__allowed_directions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Response__allowed_directions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__allowed_directions(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Response__allowed_directions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__allowed_directions(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Response__allowed_directions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

size_t basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Response__acts_as_limit(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__acts_as_limit(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__acts_as_limit(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Response__acts_as_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__acts_as_limit(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Response__acts_as_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__acts_as_limit(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Response__acts_as_limit(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_member_array[7] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "controller_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Response, controller_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "available_methods",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Response, available_methods),  // bytes offset in struct
    NULL,  // default value
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Response__available_methods,  // size() function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__available_methods,  // get_const(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__available_methods,  // get(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Response__available_methods,  // fetch(index, &value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Response__available_methods,  // assign(index, value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Response__available_methods  // resize(index) function pointer
  },
  {
    "method_descriptions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Response, method_descriptions),  // bytes offset in struct
    NULL,  // default value
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Response__method_descriptions,  // size() function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__method_descriptions,  // get_const(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__method_descriptions,  // get(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Response__method_descriptions,  // fetch(index, &value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Response__method_descriptions,  // assign(index, value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Response__method_descriptions  // resize(index) function pointer
  },
  {
    "allowed_directions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Response, allowed_directions),  // bytes offset in struct
    NULL,  // default value
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Response__allowed_directions,  // size() function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__allowed_directions,  // get_const(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__allowed_directions,  // get(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Response__allowed_directions,  // fetch(index, &value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Response__allowed_directions,  // assign(index, value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Response__allowed_directions  // resize(index) function pointer
  },
  {
    "auto_zeros_encoder",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Response, auto_zeros_encoder),  // bytes offset in struct
    NULL,  // default value
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Response__auto_zeros_encoder,  // size() function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__auto_zeros_encoder,  // get_const(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__auto_zeros_encoder,  // get(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Response__auto_zeros_encoder,  // fetch(index, &value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Response__auto_zeros_encoder,  // assign(index, value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Response__auto_zeros_encoder  // resize(index) function pointer
  },
  {
    "acts_as_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Response, acts_as_limit),  // bytes offset in struct
    NULL,  // default value
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Response__acts_as_limit,  // size() function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Response__acts_as_limit,  // get_const(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Response__acts_as_limit,  // get(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Response__acts_as_limit,  // fetch(index, &value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Response__acts_as_limit,  // assign(index, value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Response__acts_as_limit  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_members = {
  "basicmicro_ros2__srv",  // message namespace
  "GetAvailableHomingMethods_Response",  // message name
  7,  // number of fields
  sizeof(basicmicro_ros2__srv__GetAvailableHomingMethods_Response),
  false,  // has_any_key_member_
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_member_array,  // message members
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_type_support_handle = {
  0,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_basicmicro_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)() {
  if (!basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_type_support_handle.typesupport_identifier) {
    basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__rosidl_typesupport_introspection_c.h"
// already included above
// #include "basicmicro_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "basicmicro_ros2/srv/get_available_homing_methods.h"
// Member `request`
// Member `response`
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event__init(message_memory);
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_fini_function(void * message_memory)
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event__fini(message_memory);
}

size_t basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Event__request(
  const void * untyped_member)
{
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence * member =
    (const basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Event__request(
  const void * untyped_member, size_t index)
{
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence * member =
    (const basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Event__request(
  void * untyped_member, size_t index)
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence * member =
    (basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Request * item =
    ((const basicmicro_ros2__srv__GetAvailableHomingMethods_Request *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Event__request(untyped_member, index));
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request * value =
    (basicmicro_ros2__srv__GetAvailableHomingMethods_Request *)(untyped_value);
  *value = *item;
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request * item =
    ((basicmicro_ros2__srv__GetAvailableHomingMethods_Request *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Event__request(untyped_member, index));
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Request * value =
    (const basicmicro_ros2__srv__GetAvailableHomingMethods_Request *)(untyped_value);
  *item = *value;
}

bool basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Event__request(
  void * untyped_member, size_t size)
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence * member =
    (basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence *)(untyped_member);
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence__fini(member);
  return basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence__init(member, size);
}

size_t basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Event__response(
  const void * untyped_member)
{
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence * member =
    (const basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Event__response(
  const void * untyped_member, size_t index)
{
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence * member =
    (const basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Event__response(
  void * untyped_member, size_t index)
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence * member =
    (basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Response * item =
    ((const basicmicro_ros2__srv__GetAvailableHomingMethods_Response *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Event__response(untyped_member, index));
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response * value =
    (basicmicro_ros2__srv__GetAvailableHomingMethods_Response *)(untyped_value);
  *value = *item;
}

void basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response * item =
    ((basicmicro_ros2__srv__GetAvailableHomingMethods_Response *)
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Event__response(untyped_member, index));
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Response * value =
    (const basicmicro_ros2__srv__GetAvailableHomingMethods_Response *)(untyped_value);
  *item = *value;
}

bool basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Event__response(
  void * untyped_member, size_t size)
{
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence * member =
    (basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence *)(untyped_member);
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence__fini(member);
  return basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Event, info),  // bytes offset in struct
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
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Event, request),  // bytes offset in struct
    NULL,  // default value
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Event__request,  // size() function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Event__request,  // get_const(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Event__request,  // get(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Event__request,  // fetch(index, &value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Event__request,  // assign(index, value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Event__request  // resize(index) function pointer
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
    offsetof(basicmicro_ros2__srv__GetAvailableHomingMethods_Event, response),  // bytes offset in struct
    NULL,  // default value
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__size_function__GetAvailableHomingMethods_Event__response,  // size() function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_const_function__GetAvailableHomingMethods_Event__response,  // get_const(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__get_function__GetAvailableHomingMethods_Event__response,  // get(index) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__fetch_function__GetAvailableHomingMethods_Event__response,  // fetch(index, &value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__assign_function__GetAvailableHomingMethods_Event__response,  // assign(index, value) function pointer
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__resize_function__GetAvailableHomingMethods_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_members = {
  "basicmicro_ros2__srv",  // message namespace
  "GetAvailableHomingMethods_Event",  // message name
  3,  // number of fields
  sizeof(basicmicro_ros2__srv__GetAvailableHomingMethods_Event),
  false,  // has_any_key_member_
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_member_array,  // message members
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_type_support_handle = {
  0,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_basicmicro_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Event)() {
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)();
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)();
  if (!basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_type_support_handle.typesupport_identifier) {
    basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers basicmicro_ros2__srv__detail__get_available_homing_methods__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_service_members = {
  "basicmicro_ros2__srv",  // service namespace
  "GetAvailableHomingMethods",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // basicmicro_ros2__srv__detail__get_available_homing_methods__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_message_type_support_handle,
  NULL,  // response message
  // basicmicro_ros2__srv__detail__get_available_homing_methods__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_type_support_handle
  NULL  // event_message
  // basicmicro_ros2__srv__detail__get_available_homing_methods__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_type_support_handle
};


static rosidl_service_type_support_t basicmicro_ros2__srv__detail__get_available_homing_methods__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_service_type_support_handle = {
  0,
  &basicmicro_ros2__srv__detail__get_available_homing_methods__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_service_members,
  get_service_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Request_message_type_support_handle,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Response_message_type_support_handle,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    GetAvailableHomingMethods
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    GetAvailableHomingMethods
  ),
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_basicmicro_ros2
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods)(void) {
  if (!basicmicro_ros2__srv__detail__get_available_homing_methods__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_service_type_support_handle.typesupport_identifier) {
    basicmicro_ros2__srv__detail__get_available_homing_methods__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)basicmicro_ros2__srv__detail__get_available_homing_methods__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Event)()->data;
  }

  return &basicmicro_ros2__srv__detail__get_available_homing_methods__rosidl_typesupport_introspection_c__GetAvailableHomingMethods_service_type_support_handle;
}
