// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from agrobot_interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "agrobot_interfaces/srv/detail/move_servo__functions.h"
#include "agrobot_interfaces/srv/detail/move_servo__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace agrobot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveServo_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveServo_Request_type_support_ids_t;

static const _MoveServo_Request_type_support_ids_t _MoveServo_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveServo_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveServo_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveServo_Request_type_support_symbol_names_t _MoveServo_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, agrobot_interfaces, srv, MoveServo_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, agrobot_interfaces, srv, MoveServo_Request)),
  }
};

typedef struct _MoveServo_Request_type_support_data_t
{
  void * data[2];
} _MoveServo_Request_type_support_data_t;

static _MoveServo_Request_type_support_data_t _MoveServo_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveServo_Request_message_typesupport_map = {
  2,
  "agrobot_interfaces",
  &_MoveServo_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MoveServo_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MoveServo_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveServo_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveServo_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &agrobot_interfaces__srv__MoveServo_Request__get_type_hash,
  &agrobot_interfaces__srv__MoveServo_Request__get_type_description,
  &agrobot_interfaces__srv__MoveServo_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace agrobot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<agrobot_interfaces::srv::MoveServo_Request>()
{
  return &::agrobot_interfaces::srv::rosidl_typesupport_cpp::MoveServo_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, agrobot_interfaces, srv, MoveServo_Request)() {
  return get_message_type_support_handle<agrobot_interfaces::srv::MoveServo_Request>();
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
// #include "agrobot_interfaces/srv/detail/move_servo__functions.h"
// already included above
// #include "agrobot_interfaces/srv/detail/move_servo__struct.hpp"
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

namespace agrobot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveServo_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveServo_Response_type_support_ids_t;

static const _MoveServo_Response_type_support_ids_t _MoveServo_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveServo_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveServo_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveServo_Response_type_support_symbol_names_t _MoveServo_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, agrobot_interfaces, srv, MoveServo_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, agrobot_interfaces, srv, MoveServo_Response)),
  }
};

typedef struct _MoveServo_Response_type_support_data_t
{
  void * data[2];
} _MoveServo_Response_type_support_data_t;

static _MoveServo_Response_type_support_data_t _MoveServo_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveServo_Response_message_typesupport_map = {
  2,
  "agrobot_interfaces",
  &_MoveServo_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MoveServo_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MoveServo_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveServo_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveServo_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &agrobot_interfaces__srv__MoveServo_Response__get_type_hash,
  &agrobot_interfaces__srv__MoveServo_Response__get_type_description,
  &agrobot_interfaces__srv__MoveServo_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace agrobot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<agrobot_interfaces::srv::MoveServo_Response>()
{
  return &::agrobot_interfaces::srv::rosidl_typesupport_cpp::MoveServo_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, agrobot_interfaces, srv, MoveServo_Response)() {
  return get_message_type_support_handle<agrobot_interfaces::srv::MoveServo_Response>();
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
// #include "agrobot_interfaces/srv/detail/move_servo__functions.h"
// already included above
// #include "agrobot_interfaces/srv/detail/move_servo__struct.hpp"
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

namespace agrobot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveServo_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveServo_Event_type_support_ids_t;

static const _MoveServo_Event_type_support_ids_t _MoveServo_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveServo_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveServo_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveServo_Event_type_support_symbol_names_t _MoveServo_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, agrobot_interfaces, srv, MoveServo_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, agrobot_interfaces, srv, MoveServo_Event)),
  }
};

typedef struct _MoveServo_Event_type_support_data_t
{
  void * data[2];
} _MoveServo_Event_type_support_data_t;

static _MoveServo_Event_type_support_data_t _MoveServo_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveServo_Event_message_typesupport_map = {
  2,
  "agrobot_interfaces",
  &_MoveServo_Event_message_typesupport_ids.typesupport_identifier[0],
  &_MoveServo_Event_message_typesupport_symbol_names.symbol_name[0],
  &_MoveServo_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveServo_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveServo_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &agrobot_interfaces__srv__MoveServo_Event__get_type_hash,
  &agrobot_interfaces__srv__MoveServo_Event__get_type_description,
  &agrobot_interfaces__srv__MoveServo_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace agrobot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<agrobot_interfaces::srv::MoveServo_Event>()
{
  return &::agrobot_interfaces::srv::rosidl_typesupport_cpp::MoveServo_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, agrobot_interfaces, srv, MoveServo_Event)() {
  return get_message_type_support_handle<agrobot_interfaces::srv::MoveServo_Event>();
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
// #include "agrobot_interfaces/srv/detail/move_servo__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace agrobot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveServo_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveServo_type_support_ids_t;

static const _MoveServo_type_support_ids_t _MoveServo_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveServo_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveServo_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveServo_type_support_symbol_names_t _MoveServo_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, agrobot_interfaces, srv, MoveServo)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, agrobot_interfaces, srv, MoveServo)),
  }
};

typedef struct _MoveServo_type_support_data_t
{
  void * data[2];
} _MoveServo_type_support_data_t;

static _MoveServo_type_support_data_t _MoveServo_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveServo_service_typesupport_map = {
  2,
  "agrobot_interfaces",
  &_MoveServo_service_typesupport_ids.typesupport_identifier[0],
  &_MoveServo_service_typesupport_symbol_names.symbol_name[0],
  &_MoveServo_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MoveServo_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveServo_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<agrobot_interfaces::srv::MoveServo_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<agrobot_interfaces::srv::MoveServo_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<agrobot_interfaces::srv::MoveServo_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<agrobot_interfaces::srv::MoveServo>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<agrobot_interfaces::srv::MoveServo>,
  &agrobot_interfaces__srv__MoveServo__get_type_hash,
  &agrobot_interfaces__srv__MoveServo__get_type_description,
  &agrobot_interfaces__srv__MoveServo__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace agrobot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<agrobot_interfaces::srv::MoveServo>()
{
  return &::agrobot_interfaces::srv::rosidl_typesupport_cpp::MoveServo_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, agrobot_interfaces, srv, MoveServo)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<agrobot_interfaces::srv::MoveServo>();
}

#ifdef __cplusplus
}
#endif
