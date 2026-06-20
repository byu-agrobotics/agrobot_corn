// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from basicmicro_ros2:srv/MoveDistance.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "basicmicro_ros2/srv/detail/move_distance__struct.h"
#include "basicmicro_ros2/srv/detail/move_distance__type_support.h"
#include "basicmicro_ros2/srv/detail/move_distance__functions.h"
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

typedef struct _MoveDistance_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveDistance_Request_type_support_ids_t;

static const _MoveDistance_Request_type_support_ids_t _MoveDistance_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MoveDistance_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveDistance_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveDistance_Request_type_support_symbol_names_t _MoveDistance_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, MoveDistance_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, MoveDistance_Request)),
  }
};

typedef struct _MoveDistance_Request_type_support_data_t
{
  void * data[2];
} _MoveDistance_Request_type_support_data_t;

static _MoveDistance_Request_type_support_data_t _MoveDistance_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveDistance_Request_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_MoveDistance_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MoveDistance_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MoveDistance_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveDistance_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveDistance_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__MoveDistance_Request__get_type_hash,
  &basicmicro_ros2__srv__MoveDistance_Request__get_type_description,
  &basicmicro_ros2__srv__MoveDistance_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, MoveDistance_Request)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::MoveDistance_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/move_distance__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/move_distance__type_support.h"
// already included above
// #include "basicmicro_ros2/srv/detail/move_distance__functions.h"
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

typedef struct _MoveDistance_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveDistance_Response_type_support_ids_t;

static const _MoveDistance_Response_type_support_ids_t _MoveDistance_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MoveDistance_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveDistance_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveDistance_Response_type_support_symbol_names_t _MoveDistance_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, MoveDistance_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, MoveDistance_Response)),
  }
};

typedef struct _MoveDistance_Response_type_support_data_t
{
  void * data[2];
} _MoveDistance_Response_type_support_data_t;

static _MoveDistance_Response_type_support_data_t _MoveDistance_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveDistance_Response_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_MoveDistance_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MoveDistance_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MoveDistance_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveDistance_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveDistance_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__MoveDistance_Response__get_type_hash,
  &basicmicro_ros2__srv__MoveDistance_Response__get_type_description,
  &basicmicro_ros2__srv__MoveDistance_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, MoveDistance_Response)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::MoveDistance_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/move_distance__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/move_distance__type_support.h"
// already included above
// #include "basicmicro_ros2/srv/detail/move_distance__functions.h"
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

typedef struct _MoveDistance_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveDistance_Event_type_support_ids_t;

static const _MoveDistance_Event_type_support_ids_t _MoveDistance_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MoveDistance_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveDistance_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveDistance_Event_type_support_symbol_names_t _MoveDistance_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, MoveDistance_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, MoveDistance_Event)),
  }
};

typedef struct _MoveDistance_Event_type_support_data_t
{
  void * data[2];
} _MoveDistance_Event_type_support_data_t;

static _MoveDistance_Event_type_support_data_t _MoveDistance_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveDistance_Event_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_MoveDistance_Event_message_typesupport_ids.typesupport_identifier[0],
  &_MoveDistance_Event_message_typesupport_symbol_names.symbol_name[0],
  &_MoveDistance_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveDistance_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveDistance_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__MoveDistance_Event__get_type_hash,
  &basicmicro_ros2__srv__MoveDistance_Event__get_type_description,
  &basicmicro_ros2__srv__MoveDistance_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, MoveDistance_Event)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::MoveDistance_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/move_distance__type_support.h"
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
typedef struct _MoveDistance_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveDistance_type_support_ids_t;

static const _MoveDistance_type_support_ids_t _MoveDistance_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MoveDistance_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveDistance_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveDistance_type_support_symbol_names_t _MoveDistance_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, MoveDistance)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, basicmicro_ros2, srv, MoveDistance)),
  }
};

typedef struct _MoveDistance_type_support_data_t
{
  void * data[2];
} _MoveDistance_type_support_data_t;

static _MoveDistance_type_support_data_t _MoveDistance_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveDistance_service_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_MoveDistance_service_typesupport_ids.typesupport_identifier[0],
  &_MoveDistance_service_typesupport_symbol_names.symbol_name[0],
  &_MoveDistance_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MoveDistance_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveDistance_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &MoveDistance_Request_message_type_support_handle,
  &MoveDistance_Response_message_type_support_handle,
  &MoveDistance_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    MoveDistance
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    MoveDistance
  ),
  &basicmicro_ros2__srv__MoveDistance__get_type_hash,
  &basicmicro_ros2__srv__MoveDistance__get_type_description,
  &basicmicro_ros2__srv__MoveDistance__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace basicmicro_ros2

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, basicmicro_ros2, srv, MoveDistance)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_c::MoveDistance_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
