// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from basicmicro_ros2:srv/MoveDistance.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "basicmicro_ros2/srv/detail/move_distance__functions.h"
#include "basicmicro_ros2/srv/detail/move_distance__struct.hpp"
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

typedef struct _MoveDistance_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveDistance_Request_type_support_ids_t;

static const _MoveDistance_Request_type_support_ids_t _MoveDistance_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, MoveDistance_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, MoveDistance_Request)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveDistance_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__MoveDistance_Request__get_type_hash,
  &basicmicro_ros2__srv__MoveDistance_Request__get_type_description,
  &basicmicro_ros2__srv__MoveDistance_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::MoveDistance_Request>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::MoveDistance_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, MoveDistance_Request)() {
  return get_message_type_support_handle<basicmicro_ros2::srv::MoveDistance_Request>();
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
// #include "basicmicro_ros2/srv/detail/move_distance__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/move_distance__struct.hpp"
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

typedef struct _MoveDistance_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveDistance_Response_type_support_ids_t;

static const _MoveDistance_Response_type_support_ids_t _MoveDistance_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, MoveDistance_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, MoveDistance_Response)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveDistance_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__MoveDistance_Response__get_type_hash,
  &basicmicro_ros2__srv__MoveDistance_Response__get_type_description,
  &basicmicro_ros2__srv__MoveDistance_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::MoveDistance_Response>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::MoveDistance_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, MoveDistance_Response)() {
  return get_message_type_support_handle<basicmicro_ros2::srv::MoveDistance_Response>();
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
// #include "basicmicro_ros2/srv/detail/move_distance__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/move_distance__struct.hpp"
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

typedef struct _MoveDistance_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveDistance_Event_type_support_ids_t;

static const _MoveDistance_Event_type_support_ids_t _MoveDistance_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, MoveDistance_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, MoveDistance_Event)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveDistance_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__MoveDistance_Event__get_type_hash,
  &basicmicro_ros2__srv__MoveDistance_Event__get_type_description,
  &basicmicro_ros2__srv__MoveDistance_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::MoveDistance_Event>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::MoveDistance_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, MoveDistance_Event)() {
  return get_message_type_support_handle<basicmicro_ros2::srv::MoveDistance_Event>();
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
// #include "basicmicro_ros2/srv/detail/move_distance__struct.hpp"
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

typedef struct _MoveDistance_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveDistance_type_support_ids_t;

static const _MoveDistance_type_support_ids_t _MoveDistance_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, basicmicro_ros2, srv, MoveDistance)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, MoveDistance)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveDistance_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<basicmicro_ros2::srv::MoveDistance_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<basicmicro_ros2::srv::MoveDistance_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<basicmicro_ros2::srv::MoveDistance_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<basicmicro_ros2::srv::MoveDistance>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<basicmicro_ros2::srv::MoveDistance>,
  &basicmicro_ros2__srv__MoveDistance__get_type_hash,
  &basicmicro_ros2__srv__MoveDistance__get_type_description,
  &basicmicro_ros2__srv__MoveDistance__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace basicmicro_ros2

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<basicmicro_ros2::srv::MoveDistance>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_cpp::MoveDistance_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, basicmicro_ros2, srv, MoveDistance)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<basicmicro_ros2::srv::MoveDistance>();
}

#ifdef __cplusplus
}
#endif
