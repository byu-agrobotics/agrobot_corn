// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from basicmicro_ros2:srv/SetMotionStrategy.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "basicmicro_ros2/srv/detail/set_motion_strategy__struct.h"
#include "basicmicro_ros2/srv/detail/set_motion_strategy__type_support.h"
#include "basicmicro_ros2/srv/detail/set_motion_strategy__functions.h"
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

typedef struct _SetMotionStrategy_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMotionStrategy_Request_type_support_ids_t;

static const _SetMotionStrategy_Request_type_support_ids_t _SetMotionStrategy_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMotionStrategy_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMotionStrategy_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMotionStrategy_Request_type_support_symbol_names_t _SetMotionStrategy_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, SetMotionStrategy_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, SetMotionStrategy_Request)),
  }
};

typedef struct _SetMotionStrategy_Request_type_support_data_t
{
  void * data[2];
} _SetMotionStrategy_Request_type_support_data_t;

static _SetMotionStrategy_Request_type_support_data_t _SetMotionStrategy_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMotionStrategy_Request_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_SetMotionStrategy_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SetMotionStrategy_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SetMotionStrategy_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMotionStrategy_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMotionStrategy_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__SetMotionStrategy_Request__get_type_hash,
  &basicmicro_ros2__srv__SetMotionStrategy_Request__get_type_description,
  &basicmicro_ros2__srv__SetMotionStrategy_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, SetMotionStrategy_Request)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::SetMotionStrategy_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_strategy__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_strategy__type_support.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_strategy__functions.h"
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

typedef struct _SetMotionStrategy_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMotionStrategy_Response_type_support_ids_t;

static const _SetMotionStrategy_Response_type_support_ids_t _SetMotionStrategy_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMotionStrategy_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMotionStrategy_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMotionStrategy_Response_type_support_symbol_names_t _SetMotionStrategy_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, SetMotionStrategy_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, SetMotionStrategy_Response)),
  }
};

typedef struct _SetMotionStrategy_Response_type_support_data_t
{
  void * data[2];
} _SetMotionStrategy_Response_type_support_data_t;

static _SetMotionStrategy_Response_type_support_data_t _SetMotionStrategy_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMotionStrategy_Response_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_SetMotionStrategy_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SetMotionStrategy_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SetMotionStrategy_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMotionStrategy_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMotionStrategy_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__SetMotionStrategy_Response__get_type_hash,
  &basicmicro_ros2__srv__SetMotionStrategy_Response__get_type_description,
  &basicmicro_ros2__srv__SetMotionStrategy_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, SetMotionStrategy_Response)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::SetMotionStrategy_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_strategy__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_strategy__type_support.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_strategy__functions.h"
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

typedef struct _SetMotionStrategy_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMotionStrategy_Event_type_support_ids_t;

static const _SetMotionStrategy_Event_type_support_ids_t _SetMotionStrategy_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMotionStrategy_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMotionStrategy_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMotionStrategy_Event_type_support_symbol_names_t _SetMotionStrategy_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, SetMotionStrategy_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, SetMotionStrategy_Event)),
  }
};

typedef struct _SetMotionStrategy_Event_type_support_data_t
{
  void * data[2];
} _SetMotionStrategy_Event_type_support_data_t;

static _SetMotionStrategy_Event_type_support_data_t _SetMotionStrategy_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMotionStrategy_Event_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_SetMotionStrategy_Event_message_typesupport_ids.typesupport_identifier[0],
  &_SetMotionStrategy_Event_message_typesupport_symbol_names.symbol_name[0],
  &_SetMotionStrategy_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMotionStrategy_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMotionStrategy_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__SetMotionStrategy_Event__get_type_hash,
  &basicmicro_ros2__srv__SetMotionStrategy_Event__get_type_description,
  &basicmicro_ros2__srv__SetMotionStrategy_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, SetMotionStrategy_Event)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::SetMotionStrategy_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_strategy__type_support.h"
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
typedef struct _SetMotionStrategy_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMotionStrategy_type_support_ids_t;

static const _SetMotionStrategy_type_support_ids_t _SetMotionStrategy_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMotionStrategy_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMotionStrategy_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMotionStrategy_type_support_symbol_names_t _SetMotionStrategy_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, SetMotionStrategy)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, SetMotionStrategy)),
  }
};

typedef struct _SetMotionStrategy_type_support_data_t
{
  void * data[2];
} _SetMotionStrategy_type_support_data_t;

static _SetMotionStrategy_type_support_data_t _SetMotionStrategy_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMotionStrategy_service_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_SetMotionStrategy_service_typesupport_ids.typesupport_identifier[0],
  &_SetMotionStrategy_service_typesupport_symbol_names.symbol_name[0],
  &_SetMotionStrategy_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SetMotionStrategy_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMotionStrategy_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &SetMotionStrategy_Request_message_type_support_handle,
  &SetMotionStrategy_Response_message_type_support_handle,
  &SetMotionStrategy_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    SetMotionStrategy
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    SetMotionStrategy
  ),
  &basicmicro_ros2__srv__SetMotionStrategy__get_type_hash,
  &basicmicro_ros2__srv__SetMotionStrategy__get_type_description,
  &basicmicro_ros2__srv__SetMotionStrategy__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, SetMotionStrategy)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::SetMotionStrategy_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
