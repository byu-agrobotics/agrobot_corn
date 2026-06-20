// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from basicmicro_ros2:srv/GetServoStatus.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "basicmicro_ros2/srv/detail/get_servo_status__functions.h"
#include "basicmicro_ros2/srv/detail/get_servo_status__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetServoStatus_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetServoStatus_Request_type_support_ids_t;

static const _GetServoStatus_Request_type_support_ids_t _GetServoStatus_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetServoStatus_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetServoStatus_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetServoStatus_Request_type_support_symbol_names_t _GetServoStatus_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, GetServoStatus_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetServoStatus_Request)),
  }
};

typedef struct _GetServoStatus_Request_type_support_data_t
{
  void * data[2];
} _GetServoStatus_Request_type_support_data_t;

static _GetServoStatus_Request_type_support_data_t _GetServoStatus_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetServoStatus_Request_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_GetServoStatus_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetServoStatus_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetServoStatus_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetServoStatus_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetServoStatus_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetServoStatus_Request__get_type_hash,
  &basicmicro_ros2__srv__GetServoStatus_Request__get_type_description,
  &basicmicro_ros2__srv__GetServoStatus_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::GetServoStatus_Request>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::GetServoStatus_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, GetServoStatus_Request)() {
  return get_message_type_support_handle<basicmicro_ros2::srv::GetServoStatus_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetServoStatus_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetServoStatus_Response_type_support_ids_t;

static const _GetServoStatus_Response_type_support_ids_t _GetServoStatus_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetServoStatus_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetServoStatus_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetServoStatus_Response_type_support_symbol_names_t _GetServoStatus_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, GetServoStatus_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetServoStatus_Response)),
  }
};

typedef struct _GetServoStatus_Response_type_support_data_t
{
  void * data[2];
} _GetServoStatus_Response_type_support_data_t;

static _GetServoStatus_Response_type_support_data_t _GetServoStatus_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetServoStatus_Response_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_GetServoStatus_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetServoStatus_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetServoStatus_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetServoStatus_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetServoStatus_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetServoStatus_Response__get_type_hash,
  &basicmicro_ros2__srv__GetServoStatus_Response__get_type_description,
  &basicmicro_ros2__srv__GetServoStatus_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::GetServoStatus_Response>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::GetServoStatus_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, GetServoStatus_Response)() {
  return get_message_type_support_handle<basicmicro_ros2::srv::GetServoStatus_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetServoStatus_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetServoStatus_Event_type_support_ids_t;

static const _GetServoStatus_Event_type_support_ids_t _GetServoStatus_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetServoStatus_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetServoStatus_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetServoStatus_Event_type_support_symbol_names_t _GetServoStatus_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, GetServoStatus_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetServoStatus_Event)),
  }
};

typedef struct _GetServoStatus_Event_type_support_data_t
{
  void * data[2];
} _GetServoStatus_Event_type_support_data_t;

static _GetServoStatus_Event_type_support_data_t _GetServoStatus_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetServoStatus_Event_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_GetServoStatus_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GetServoStatus_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GetServoStatus_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetServoStatus_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetServoStatus_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetServoStatus_Event__get_type_hash,
  &basicmicro_ros2__srv__GetServoStatus_Event__get_type_description,
  &basicmicro_ros2__srv__GetServoStatus_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::GetServoStatus_Event>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::GetServoStatus_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, GetServoStatus_Event)() {
  return get_message_type_support_handle<basicmicro_ros2::srv::GetServoStatus_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "basicmicro_ros2/srv/detail/get_servo_status__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetServoStatus_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetServoStatus_type_support_ids_t;

static const _GetServoStatus_type_support_ids_t _GetServoStatus_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetServoStatus_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetServoStatus_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetServoStatus_type_support_symbol_names_t _GetServoStatus_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, GetServoStatus)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetServoStatus)),
  }
};

typedef struct _GetServoStatus_type_support_data_t
{
  void * data[2];
} _GetServoStatus_type_support_data_t;

static _GetServoStatus_type_support_data_t _GetServoStatus_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetServoStatus_service_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_GetServoStatus_service_typesupport_ids.typesupport_identifier[0],
  &_GetServoStatus_service_typesupport_symbol_names.symbol_name[0],
  &_GetServoStatus_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetServoStatus_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetServoStatus_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetServoStatus_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetServoStatus_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetServoStatus_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<basicmicro_ros2::srv::GetServoStatus>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<basicmicro_ros2::srv::GetServoStatus>,
  &basicmicro_ros2__srv__GetServoStatus__get_type_hash,
  &basicmicro_ros2__srv__GetServoStatus__get_type_description,
  &basicmicro_ros2__srv__GetServoStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<basicmicro_ros2::srv::GetServoStatus>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::GetServoStatus_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, GetServoStatus)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<basicmicro_ros2::srv::GetServoStatus>();
}

#ifdef __cplusplus
}
#endif
