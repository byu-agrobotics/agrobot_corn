// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from basicmicro_ros2:srv/PerformHoming.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "basicmicro_ros2/srv/detail/perform_homing__struct.h"
#include "basicmicro_ros2/srv/detail/perform_homing__type_support.h"
#include "basicmicro_ros2/srv/detail/perform_homing__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _PerformHoming_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PerformHoming_Request_type_support_ids_t;

static const _PerformHoming_Request_type_support_ids_t _PerformHoming_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PerformHoming_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PerformHoming_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PerformHoming_Request_type_support_symbol_names_t _PerformHoming_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, PerformHoming_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, PerformHoming_Request)),
  }
};

typedef struct _PerformHoming_Request_type_support_data_t
{
  void * data[2];
} _PerformHoming_Request_type_support_data_t;

static _PerformHoming_Request_type_support_data_t _PerformHoming_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PerformHoming_Request_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_PerformHoming_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PerformHoming_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PerformHoming_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PerformHoming_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PerformHoming_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__PerformHoming_Request__get_type_hash,
  &basicmicro_ros2__srv__PerformHoming_Request__get_type_description,
  &basicmicro_ros2__srv__PerformHoming_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, PerformHoming_Request)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::PerformHoming_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/perform_homing__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/perform_homing__type_support.h"
// already included above
// #include "basicmicro_ros2/srv/detail/perform_homing__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _PerformHoming_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PerformHoming_Response_type_support_ids_t;

static const _PerformHoming_Response_type_support_ids_t _PerformHoming_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PerformHoming_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PerformHoming_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PerformHoming_Response_type_support_symbol_names_t _PerformHoming_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, PerformHoming_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, PerformHoming_Response)),
  }
};

typedef struct _PerformHoming_Response_type_support_data_t
{
  void * data[2];
} _PerformHoming_Response_type_support_data_t;

static _PerformHoming_Response_type_support_data_t _PerformHoming_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PerformHoming_Response_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_PerformHoming_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PerformHoming_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PerformHoming_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PerformHoming_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PerformHoming_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__PerformHoming_Response__get_type_hash,
  &basicmicro_ros2__srv__PerformHoming_Response__get_type_description,
  &basicmicro_ros2__srv__PerformHoming_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, PerformHoming_Response)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::PerformHoming_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/perform_homing__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/perform_homing__type_support.h"
// already included above
// #include "basicmicro_ros2/srv/detail/perform_homing__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _PerformHoming_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PerformHoming_Event_type_support_ids_t;

static const _PerformHoming_Event_type_support_ids_t _PerformHoming_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PerformHoming_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PerformHoming_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PerformHoming_Event_type_support_symbol_names_t _PerformHoming_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, PerformHoming_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, PerformHoming_Event)),
  }
};

typedef struct _PerformHoming_Event_type_support_data_t
{
  void * data[2];
} _PerformHoming_Event_type_support_data_t;

static _PerformHoming_Event_type_support_data_t _PerformHoming_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PerformHoming_Event_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_PerformHoming_Event_message_typesupport_ids.typesupport_identifier[0],
  &_PerformHoming_Event_message_typesupport_symbol_names.symbol_name[0],
  &_PerformHoming_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PerformHoming_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PerformHoming_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__PerformHoming_Event__get_type_hash,
  &basicmicro_ros2__srv__PerformHoming_Event__get_type_description,
  &basicmicro_ros2__srv__PerformHoming_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, PerformHoming_Event)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::PerformHoming_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/perform_homing__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _PerformHoming_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PerformHoming_type_support_ids_t;

static const _PerformHoming_type_support_ids_t _PerformHoming_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PerformHoming_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PerformHoming_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PerformHoming_type_support_symbol_names_t _PerformHoming_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, PerformHoming)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, PerformHoming)),
  }
};

typedef struct _PerformHoming_type_support_data_t
{
  void * data[2];
} _PerformHoming_type_support_data_t;

static _PerformHoming_type_support_data_t _PerformHoming_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PerformHoming_service_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_PerformHoming_service_typesupport_ids.typesupport_identifier[0],
  &_PerformHoming_service_typesupport_symbol_names.symbol_name[0],
  &_PerformHoming_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PerformHoming_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PerformHoming_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &PerformHoming_Request_message_type_support_handle,
  &PerformHoming_Response_message_type_support_handle,
  &PerformHoming_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    PerformHoming
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    PerformHoming
  ),
  &basicmicro_ros2__srv__PerformHoming__get_type_hash,
  &basicmicro_ros2__srv__PerformHoming__get_type_description,
  &basicmicro_ros2__srv__PerformHoming__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, PerformHoming)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::PerformHoming_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
