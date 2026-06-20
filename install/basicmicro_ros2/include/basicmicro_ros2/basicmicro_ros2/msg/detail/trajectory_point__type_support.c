// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from basicmicro_ros2:msg/TrajectoryPoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "basicmicro_ros2/msg/detail/trajectory_point__rosidl_typesupport_introspection_c.h"
#include "basicmicro_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "basicmicro_ros2/msg/detail/trajectory_point__functions.h"
#include "basicmicro_ros2/msg/detail/trajectory_point__struct.h"


// Include directives for member types
// Member `command_type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  basicmicro_ros2__msg__TrajectoryPoint__init(message_memory);
}

void basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_fini_function(void * message_memory)
{
  basicmicro_ros2__msg__TrajectoryPoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_message_member_array[9] = {
  {
    "command_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__TrajectoryPoint, command_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__TrajectoryPoint, left_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__TrajectoryPoint, right_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__TrajectoryPoint, left_position),  // bytes offset in struct
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
    offsetof(basicmicro_ros2__msg__TrajectoryPoint, right_position),  // bytes offset in struct
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
    offsetof(basicmicro_ros2__msg__TrajectoryPoint, deceleration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__TrajectoryPoint, speed),  // bytes offset in struct
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
    offsetof(basicmicro_ros2__msg__TrajectoryPoint, acceleration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "duration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2__msg__TrajectoryPoint, duration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_message_members = {
  "basicmicro_ros2__msg",  // message namespace
  "TrajectoryPoint",  // message name
  9,  // number of fields
  sizeof(basicmicro_ros2__msg__TrajectoryPoint),
  false,  // has_any_key_member_
  basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_message_member_array,  // message members
  basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_init_function,  // function to initialize message memory (memory has to be allocated)
  basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_message_type_support_handle = {
  0,
  &basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__msg__TrajectoryPoint__get_type_hash,
  &basicmicro_ros2__msg__TrajectoryPoint__get_type_description,
  &basicmicro_ros2__msg__TrajectoryPoint__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_basicmicro_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, msg, TrajectoryPoint)() {
  if (!basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_message_type_support_handle.typesupport_identifier) {
    basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &basicmicro_ros2__msg__TrajectoryPoint__rosidl_typesupport_introspection_c__TrajectoryPoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
