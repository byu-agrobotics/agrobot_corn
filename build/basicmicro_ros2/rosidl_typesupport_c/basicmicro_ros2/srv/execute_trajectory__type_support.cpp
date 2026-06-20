// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from basicmicro_ros2:srv/ExecuteTrajectory.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "basicmicro_ros2/srv/detail/execute_trajectory__struct.h"
#include "basicmicro_ros2/srv/detail/execute_trajectory__type_support.h"
#include "basicmicro_ros2/srv/detail/execute_trajectory__functions.h"
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

typedef struct _ExecuteTrajectory_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExecuteTrajectory_Request_type_support_ids_t;

static const _ExecuteTrajectory_Request_type_support_ids_t _ExecuteTrajectory_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ExecuteTrajectory_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExecuteTrajectory_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExecuteTrajectory_Request_type_support_symbol_names_t _ExecuteTrajectory_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, ExecuteTrajectory_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, ExecuteTrajectory_Request)),
  }
};

typedef struct _ExecuteTrajectory_Request_type_support_data_t
{
  void * data[2];
} _ExecuteTrajectory_Request_type_support_data_t;

static _ExecuteTrajectory_Request_type_support_data_t _ExecuteTrajectory_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExecuteTrajectory_Request_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_ExecuteTrajectory_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ExecuteTrajectory_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ExecuteTrajectory_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExecuteTrajectory_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExecuteTrajectory_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__ExecuteTrajectory_Request__get_type_hash,
  &basicmicro_ros2__srv__ExecuteTrajectory_Request__get_type_description,
  &basicmicro_ros2__srv__ExecuteTrajectory_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, ExecuteTrajectory_Request)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::ExecuteTrajectory_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/execute_trajectory__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/execute_trajectory__type_support.h"
// already included above
// #include "basicmicro_ros2/srv/detail/execute_trajectory__functions.h"
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

typedef struct _ExecuteTrajectory_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExecuteTrajectory_Response_type_support_ids_t;

static const _ExecuteTrajectory_Response_type_support_ids_t _ExecuteTrajectory_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ExecuteTrajectory_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExecuteTrajectory_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExecuteTrajectory_Response_type_support_symbol_names_t _ExecuteTrajectory_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, ExecuteTrajectory_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, ExecuteTrajectory_Response)),
  }
};

typedef struct _ExecuteTrajectory_Response_type_support_data_t
{
  void * data[2];
} _ExecuteTrajectory_Response_type_support_data_t;

static _ExecuteTrajectory_Response_type_support_data_t _ExecuteTrajectory_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExecuteTrajectory_Response_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_ExecuteTrajectory_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ExecuteTrajectory_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ExecuteTrajectory_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExecuteTrajectory_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExecuteTrajectory_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__ExecuteTrajectory_Response__get_type_hash,
  &basicmicro_ros2__srv__ExecuteTrajectory_Response__get_type_description,
  &basicmicro_ros2__srv__ExecuteTrajectory_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, ExecuteTrajectory_Response)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::ExecuteTrajectory_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/execute_trajectory__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/execute_trajectory__type_support.h"
// already included above
// #include "basicmicro_ros2/srv/detail/execute_trajectory__functions.h"
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

typedef struct _ExecuteTrajectory_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExecuteTrajectory_Event_type_support_ids_t;

static const _ExecuteTrajectory_Event_type_support_ids_t _ExecuteTrajectory_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ExecuteTrajectory_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExecuteTrajectory_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExecuteTrajectory_Event_type_support_symbol_names_t _ExecuteTrajectory_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, ExecuteTrajectory_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, ExecuteTrajectory_Event)),
  }
};

typedef struct _ExecuteTrajectory_Event_type_support_data_t
{
  void * data[2];
} _ExecuteTrajectory_Event_type_support_data_t;

static _ExecuteTrajectory_Event_type_support_data_t _ExecuteTrajectory_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExecuteTrajectory_Event_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_ExecuteTrajectory_Event_message_typesupport_ids.typesupport_identifier[0],
  &_ExecuteTrajectory_Event_message_typesupport_symbol_names.symbol_name[0],
  &_ExecuteTrajectory_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExecuteTrajectory_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExecuteTrajectory_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__ExecuteTrajectory_Event__get_type_hash,
  &basicmicro_ros2__srv__ExecuteTrajectory_Event__get_type_description,
  &basicmicro_ros2__srv__ExecuteTrajectory_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, ExecuteTrajectory_Event)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::ExecuteTrajectory_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/execute_trajectory__type_support.h"
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
typedef struct _ExecuteTrajectory_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExecuteTrajectory_type_support_ids_t;

static const _ExecuteTrajectory_type_support_ids_t _ExecuteTrajectory_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ExecuteTrajectory_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExecuteTrajectory_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExecuteTrajectory_type_support_symbol_names_t _ExecuteTrajectory_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, ExecuteTrajectory)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, ExecuteTrajectory)),
  }
};

typedef struct _ExecuteTrajectory_type_support_data_t
{
  void * data[2];
} _ExecuteTrajectory_type_support_data_t;

static _ExecuteTrajectory_type_support_data_t _ExecuteTrajectory_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExecuteTrajectory_service_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_ExecuteTrajectory_service_typesupport_ids.typesupport_identifier[0],
  &_ExecuteTrajectory_service_typesupport_symbol_names.symbol_name[0],
  &_ExecuteTrajectory_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ExecuteTrajectory_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExecuteTrajectory_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &ExecuteTrajectory_Request_message_type_support_handle,
  &ExecuteTrajectory_Response_message_type_support_handle,
  &ExecuteTrajectory_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    ExecuteTrajectory
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    ExecuteTrajectory
  ),
  &basicmicro_ros2__srv__ExecuteTrajectory__get_type_hash,
  &basicmicro_ros2__srv__ExecuteTrajectory__get_type_description,
  &basicmicro_ros2__srv__ExecuteTrajectory__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, ExecuteTrajectory)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::ExecuteTrajectory_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
