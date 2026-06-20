// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from basicmicro_ros2:srv/GetAvailableHomingMethods.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
#include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.hpp"
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

typedef struct _GetAvailableHomingMethods_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetAvailableHomingMethods_Request_type_support_ids_t;

static const _GetAvailableHomingMethods_Request_type_support_ids_t _GetAvailableHomingMethods_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetAvailableHomingMethods_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetAvailableHomingMethods_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetAvailableHomingMethods_Request_type_support_symbol_names_t _GetAvailableHomingMethods_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)),
  }
};

typedef struct _GetAvailableHomingMethods_Request_type_support_data_t
{
  void * data[2];
} _GetAvailableHomingMethods_Request_type_support_data_t;

static _GetAvailableHomingMethods_Request_type_support_data_t _GetAvailableHomingMethods_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetAvailableHomingMethods_Request_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_GetAvailableHomingMethods_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetAvailableHomingMethods_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetAvailableHomingMethods_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetAvailableHomingMethods_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetAvailableHomingMethods_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::GetAvailableHomingMethods_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)() {
  return get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>();
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
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.hpp"
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

typedef struct _GetAvailableHomingMethods_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetAvailableHomingMethods_Response_type_support_ids_t;

static const _GetAvailableHomingMethods_Response_type_support_ids_t _GetAvailableHomingMethods_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetAvailableHomingMethods_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetAvailableHomingMethods_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetAvailableHomingMethods_Response_type_support_symbol_names_t _GetAvailableHomingMethods_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)),
  }
};

typedef struct _GetAvailableHomingMethods_Response_type_support_data_t
{
  void * data[2];
} _GetAvailableHomingMethods_Response_type_support_data_t;

static _GetAvailableHomingMethods_Response_type_support_data_t _GetAvailableHomingMethods_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetAvailableHomingMethods_Response_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_GetAvailableHomingMethods_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetAvailableHomingMethods_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetAvailableHomingMethods_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetAvailableHomingMethods_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetAvailableHomingMethods_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::GetAvailableHomingMethods_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)() {
  return get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>();
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
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.hpp"
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

typedef struct _GetAvailableHomingMethods_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetAvailableHomingMethods_Event_type_support_ids_t;

static const _GetAvailableHomingMethods_Event_type_support_ids_t _GetAvailableHomingMethods_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetAvailableHomingMethods_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetAvailableHomingMethods_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetAvailableHomingMethods_Event_type_support_symbol_names_t _GetAvailableHomingMethods_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Event)),
  }
};

typedef struct _GetAvailableHomingMethods_Event_type_support_data_t
{
  void * data[2];
} _GetAvailableHomingMethods_Event_type_support_data_t;

static _GetAvailableHomingMethods_Event_type_support_data_t _GetAvailableHomingMethods_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetAvailableHomingMethods_Event_message_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_GetAvailableHomingMethods_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GetAvailableHomingMethods_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GetAvailableHomingMethods_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetAvailableHomingMethods_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetAvailableHomingMethods_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Event>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::GetAvailableHomingMethods_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Event)() {
  return get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Event>();
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
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.hpp"
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

typedef struct _GetAvailableHomingMethods_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetAvailableHomingMethods_type_support_ids_t;

static const _GetAvailableHomingMethods_type_support_ids_t _GetAvailableHomingMethods_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetAvailableHomingMethods_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetAvailableHomingMethods_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetAvailableHomingMethods_type_support_symbol_names_t _GetAvailableHomingMethods_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods)),
  }
};

typedef struct _GetAvailableHomingMethods_type_support_data_t
{
  void * data[2];
} _GetAvailableHomingMethods_type_support_data_t;

static _GetAvailableHomingMethods_type_support_data_t _GetAvailableHomingMethods_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetAvailableHomingMethods_service_typesupport_map = {
  2,
  "basicmicro_ros2",
  &_GetAvailableHomingMethods_service_typesupport_ids.typesupport_identifier[0],
  &_GetAvailableHomingMethods_service_typesupport_symbol_names.symbol_name[0],
  &_GetAvailableHomingMethods_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetAvailableHomingMethods_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetAvailableHomingMethods_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<basicmicro_ros2::srv::GetAvailableHomingMethods>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<basicmicro_ros2::srv::GetAvailableHomingMethods>,
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::GetAvailableHomingMethods_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods>();
}

#ifdef __cplusplus
}
#endif
