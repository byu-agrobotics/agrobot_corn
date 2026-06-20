// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from basicmicro_ros2:srv/SetMotionParameters.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "basicmicro_ros2/srv/detail/set_motion_parameters__struct.h"
#include "basicmicro_ros2/srv/detail/set_motion_parameters__type_support.h"
#include "basicmicro_ros2/srv/detail/set_motion_parameters__functions.h"
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

typedef struct _SetMotionParameters_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMotionParameters_Request_type_support_ids_t;

static const _SetMotionParameters_Request_type_support_ids_t _SetMotionParameters_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMotionParameters_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMotionParameters_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMotionParameters_Request_type_support_symbol_names_t _SetMotionParameters_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, SetMotionParameters_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, SetMotionParameters_Request)),
  }
};

typedef struct _SetMotionParameters_Request_type_support_data_t
{
  void * data[2];
} _SetMotionParameters_Request_type_support_data_t;

static _SetMotionParameters_Request_type_support_data_t _SetMotionParameters_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMotionParameters_Request_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_SetMotionParameters_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SetMotionParameters_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SetMotionParameters_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMotionParameters_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMotionParameters_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__SetMotionParameters_Request__get_type_hash,
  &basicmicro_ros2__srv__SetMotionParameters_Request__get_type_description,
  &basicmicro_ros2__srv__SetMotionParameters_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, SetMotionParameters_Request)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::SetMotionParameters_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_parameters__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_parameters__type_support.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_parameters__functions.h"
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

typedef struct _SetMotionParameters_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMotionParameters_Response_type_support_ids_t;

static const _SetMotionParameters_Response_type_support_ids_t _SetMotionParameters_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMotionParameters_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMotionParameters_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMotionParameters_Response_type_support_symbol_names_t _SetMotionParameters_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, SetMotionParameters_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, SetMotionParameters_Response)),
  }
};

typedef struct _SetMotionParameters_Response_type_support_data_t
{
  void * data[2];
} _SetMotionParameters_Response_type_support_data_t;

static _SetMotionParameters_Response_type_support_data_t _SetMotionParameters_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMotionParameters_Response_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_SetMotionParameters_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SetMotionParameters_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SetMotionParameters_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMotionParameters_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMotionParameters_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__SetMotionParameters_Response__get_type_hash,
  &basicmicro_ros2__srv__SetMotionParameters_Response__get_type_description,
  &basicmicro_ros2__srv__SetMotionParameters_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, SetMotionParameters_Response)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::SetMotionParameters_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_parameters__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_parameters__type_support.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_parameters__functions.h"
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

typedef struct _SetMotionParameters_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMotionParameters_Event_type_support_ids_t;

static const _SetMotionParameters_Event_type_support_ids_t _SetMotionParameters_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMotionParameters_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMotionParameters_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMotionParameters_Event_type_support_symbol_names_t _SetMotionParameters_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, SetMotionParameters_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, SetMotionParameters_Event)),
  }
};

typedef struct _SetMotionParameters_Event_type_support_data_t
{
  void * data[2];
} _SetMotionParameters_Event_type_support_data_t;

static _SetMotionParameters_Event_type_support_data_t _SetMotionParameters_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMotionParameters_Event_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_SetMotionParameters_Event_message_typesupport_ids.typesupport_identifier[0],
  &_SetMotionParameters_Event_message_typesupport_symbol_names.symbol_name[0],
  &_SetMotionParameters_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetMotionParameters_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMotionParameters_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__SetMotionParameters_Event__get_type_hash,
  &basicmicro_ros2__srv__SetMotionParameters_Event__get_type_description,
  &basicmicro_ros2__srv__SetMotionParameters_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, SetMotionParameters_Event)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::SetMotionParameters_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/set_motion_parameters__type_support.h"
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
typedef struct _SetMotionParameters_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetMotionParameters_type_support_ids_t;

static const _SetMotionParameters_type_support_ids_t _SetMotionParameters_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetMotionParameters_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetMotionParameters_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetMotionParameters_type_support_symbol_names_t _SetMotionParameters_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, SetMotionParameters)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, SetMotionParameters)),
  }
};

typedef struct _SetMotionParameters_type_support_data_t
{
  void * data[2];
} _SetMotionParameters_type_support_data_t;

static _SetMotionParameters_type_support_data_t _SetMotionParameters_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetMotionParameters_service_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_SetMotionParameters_service_typesupport_ids.typesupport_identifier[0],
  &_SetMotionParameters_service_typesupport_symbol_names.symbol_name[0],
  &_SetMotionParameters_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SetMotionParameters_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetMotionParameters_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &SetMotionParameters_Request_message_type_support_handle,
  &SetMotionParameters_Response_message_type_support_handle,
  &SetMotionParameters_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    SetMotionParameters
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    SetMotionParameters
  ),
  &basicmicro_ros2__srv__SetMotionParameters__get_type_hash,
  &basicmicro_ros2__srv__SetMotionParameters__get_type_description,
  &basicmicro_ros2__srv__SetMotionParameters__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, SetMotionParameters)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::SetMotionParameters_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
