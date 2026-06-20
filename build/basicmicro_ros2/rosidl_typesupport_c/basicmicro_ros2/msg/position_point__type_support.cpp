// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from basicmicro_ros2:msg/PositionPoint.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "basicmicro_ros2/msg/detail/position_point__struct.h"
#include "basicmicro_ros2/msg/detail/position_point__type_support.h"
#include "basicmicro_ros2/msg/detail/position_point__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace basicmicro_ros2
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _PositionPoint_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PositionPoint_type_support_ids_t;

static const _PositionPoint_type_support_ids_t _PositionPoint_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PositionPoint_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PositionPoint_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PositionPoint_type_support_symbol_names_t _PositionPoint_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, msg, PositionPoint)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, msg, PositionPoint)),
  }
};

typedef struct _PositionPoint_type_support_data_t
{
  void * data[2];
} _PositionPoint_type_support_data_t;

static _PositionPoint_type_support_data_t _PositionPoint_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PositionPoint_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_PositionPoint_message_typesupport_ids.typesupport_identifier[0],
  &_PositionPoint_message_typesupport_symbol_names.symbol_name[0],
  &_PositionPoint_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PositionPoint_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PositionPoint_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__msg__PositionPoint__get_type_hash,
  &basicmicro_ros2__msg__PositionPoint__get_type_description,
  &basicmicro_ros2__msg__PositionPoint__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, msg, PositionPoint)() {
  return &::basicmicro_ros2::msg::rosidl_typesupport_c::PositionPoint_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
