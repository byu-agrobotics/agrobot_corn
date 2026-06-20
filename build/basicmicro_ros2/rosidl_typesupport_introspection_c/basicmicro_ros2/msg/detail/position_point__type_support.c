// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from basicmicro_ros2:msg/PositionPoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "basicmicro_ros2/msg/detail/position_point__rosidl_typesupport_introspection_c.h"
#include "basicmicro_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "basicmicro_ros2/msg/detail/position_point__functions.h"
#include "basicmicro_ros2/msg/detail/position_point__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  basicmicro_ros2__msg__PositionPoint__init(message_memory);
}

void basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_fini_function(void * message_memory)
{
  basicmicro_ros2__msg__PositionPoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_message_member_array[5] = {
  {
    "left_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__PositionPoint, left_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__PositionPoint, right_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__PositionPoint, max_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__PositionPoint, acceleration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "deceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__PositionPoint, deceleration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_message_members = {
  "basicmicro_ros2__msg",  // message namespace
  "PositionPoint",  // message name
  5,  // number of fields
  sizeof(basicmicro_ros2__msg__PositionPoint),
  false,  // has_any_key_member_
  basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_message_member_array,  // message members
  basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_init_function,  // function to initialize message memory (memory has to be allocated)
  basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_message_type_support_handle = {
  0,
  &basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__msg__PositionPoint__get_type_hash,
  &basicmicro_ros2__msg__PositionPoint__get_type_description,
  &basicmicro_ros2__msg__PositionPoint__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_basicmicro_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, msg, PositionPoint)() {
  if (!basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_message_type_support_handle.typesupport_identifier) {
    basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &basicmicro_ros2__msg__PositionPoint__rosidl_typesupport_introspection_c__PositionPoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
