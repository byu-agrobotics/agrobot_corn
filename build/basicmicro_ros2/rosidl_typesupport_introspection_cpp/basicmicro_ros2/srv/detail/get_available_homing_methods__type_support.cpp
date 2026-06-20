// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from basicmicro_ros2:srv/GetAvailableHomingMethods.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
#include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetAvailableHomingMethods_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) basicmicro_ros2::srv::GetAvailableHomingMethods_Request(_init);
}

void GetAvailableHomingMethods_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<basicmicro_ros2::srv::GetAvailableHomingMethods_Request *>(message_memory);
  typed_message->~GetAvailableHomingMethods_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetAvailableHomingMethods_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetAvailableHomingMethods_Request_message_members = {
  "basicmicro_ros2::srv",  // message namespace
  "GetAvailableHomingMethods_Request",  // message name
  1,  // number of fields
  sizeof(basicmicro_ros2::srv::GetAvailableHomingMethods_Request),
  false,  // has_any_key_member_
  GetAvailableHomingMethods_Request_message_member_array,  // message members
  GetAvailableHomingMethods_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetAvailableHomingMethods_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetAvailableHomingMethods_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetAvailableHomingMethods_Request_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace basicmicro_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_introspection_cpp::GetAvailableHomingMethods_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_introspection_cpp::GetAvailableHomingMethods_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetAvailableHomingMethods_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) basicmicro_ros2::srv::GetAvailableHomingMethods_Response(_init);
}

void GetAvailableHomingMethods_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<basicmicro_ros2::srv::GetAvailableHomingMethods_Response *>(message_memory);
  typed_message->~GetAvailableHomingMethods_Response();
}

size_t size_function__GetAvailableHomingMethods_Response__available_methods(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetAvailableHomingMethods_Response__available_methods(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__GetAvailableHomingMethods_Response__available_methods(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetAvailableHomingMethods_Response__available_methods(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__GetAvailableHomingMethods_Response__available_methods(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__GetAvailableHomingMethods_Response__available_methods(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__GetAvailableHomingMethods_Response__available_methods(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__GetAvailableHomingMethods_Response__available_methods(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetAvailableHomingMethods_Response__method_descriptions(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetAvailableHomingMethods_Response__method_descriptions(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__GetAvailableHomingMethods_Response__method_descriptions(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetAvailableHomingMethods_Response__method_descriptions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__GetAvailableHomingMethods_Response__method_descriptions(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__GetAvailableHomingMethods_Response__method_descriptions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__GetAvailableHomingMethods_Response__method_descriptions(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__GetAvailableHomingMethods_Response__method_descriptions(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetAvailableHomingMethods_Response__allowed_directions(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetAvailableHomingMethods_Response__allowed_directions(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__GetAvailableHomingMethods_Response__allowed_directions(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetAvailableHomingMethods_Response__allowed_directions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__GetAvailableHomingMethods_Response__allowed_directions(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__GetAvailableHomingMethods_Response__allowed_directions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__GetAvailableHomingMethods_Response__allowed_directions(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__GetAvailableHomingMethods_Response__allowed_directions(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__GetAvailableHomingMethods_Response__auto_zeros_encoder(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetAvailableHomingMethods_Response__acts_as_limit(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__GetAvailableHomingMethods_Response__acts_as_limit(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__GetAvailableHomingMethods_Response__acts_as_limit(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__GetAvailableHomingMethods_Response__acts_as_limit(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetAvailableHomingMethods_Response_message_member_array[7] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "controller_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Response, controller_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "available_methods",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Response, available_methods),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetAvailableHomingMethods_Response__available_methods,  // size() function pointer
    get_const_function__GetAvailableHomingMethods_Response__available_methods,  // get_const(index) function pointer
    get_function__GetAvailableHomingMethods_Response__available_methods,  // get(index) function pointer
    fetch_function__GetAvailableHomingMethods_Response__available_methods,  // fetch(index, &value) function pointer
    assign_function__GetAvailableHomingMethods_Response__available_methods,  // assign(index, value) function pointer
    resize_function__GetAvailableHomingMethods_Response__available_methods  // resize(index) function pointer
  },
  {
    "method_descriptions",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Response, method_descriptions),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetAvailableHomingMethods_Response__method_descriptions,  // size() function pointer
    get_const_function__GetAvailableHomingMethods_Response__method_descriptions,  // get_const(index) function pointer
    get_function__GetAvailableHomingMethods_Response__method_descriptions,  // get(index) function pointer
    fetch_function__GetAvailableHomingMethods_Response__method_descriptions,  // fetch(index, &value) function pointer
    assign_function__GetAvailableHomingMethods_Response__method_descriptions,  // assign(index, value) function pointer
    resize_function__GetAvailableHomingMethods_Response__method_descriptions  // resize(index) function pointer
  },
  {
    "allowed_directions",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Response, allowed_directions),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetAvailableHomingMethods_Response__allowed_directions,  // size() function pointer
    get_const_function__GetAvailableHomingMethods_Response__allowed_directions,  // get_const(index) function pointer
    get_function__GetAvailableHomingMethods_Response__allowed_directions,  // get(index) function pointer
    fetch_function__GetAvailableHomingMethods_Response__allowed_directions,  // fetch(index, &value) function pointer
    assign_function__GetAvailableHomingMethods_Response__allowed_directions,  // assign(index, value) function pointer
    resize_function__GetAvailableHomingMethods_Response__allowed_directions  // resize(index) function pointer
  },
  {
    "auto_zeros_encoder",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Response, auto_zeros_encoder),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetAvailableHomingMethods_Response__auto_zeros_encoder,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__GetAvailableHomingMethods_Response__auto_zeros_encoder,  // fetch(index, &value) function pointer
    assign_function__GetAvailableHomingMethods_Response__auto_zeros_encoder,  // assign(index, value) function pointer
    resize_function__GetAvailableHomingMethods_Response__auto_zeros_encoder  // resize(index) function pointer
  },
  {
    "acts_as_limit",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Response, acts_as_limit),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetAvailableHomingMethods_Response__acts_as_limit,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__GetAvailableHomingMethods_Response__acts_as_limit,  // fetch(index, &value) function pointer
    assign_function__GetAvailableHomingMethods_Response__acts_as_limit,  // assign(index, value) function pointer
    resize_function__GetAvailableHomingMethods_Response__acts_as_limit  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetAvailableHomingMethods_Response_message_members = {
  "basicmicro_ros2::srv",  // message namespace
  "GetAvailableHomingMethods_Response",  // message name
  7,  // number of fields
  sizeof(basicmicro_ros2::srv::GetAvailableHomingMethods_Response),
  false,  // has_any_key_member_
  GetAvailableHomingMethods_Response_message_member_array,  // message members
  GetAvailableHomingMethods_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetAvailableHomingMethods_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetAvailableHomingMethods_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetAvailableHomingMethods_Response_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace basicmicro_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_introspection_cpp::GetAvailableHomingMethods_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_introspection_cpp::GetAvailableHomingMethods_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetAvailableHomingMethods_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) basicmicro_ros2::srv::GetAvailableHomingMethods_Event(_init);
}

void GetAvailableHomingMethods_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<basicmicro_ros2::srv::GetAvailableHomingMethods_Event *>(message_memory);
  typed_message->~GetAvailableHomingMethods_Event();
}

size_t size_function__GetAvailableHomingMethods_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<basicmicro_ros2::srv::GetAvailableHomingMethods_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetAvailableHomingMethods_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<basicmicro_ros2::srv::GetAvailableHomingMethods_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__GetAvailableHomingMethods_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<basicmicro_ros2::srv::GetAvailableHomingMethods_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetAvailableHomingMethods_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const basicmicro_ros2::srv::GetAvailableHomingMethods_Request *>(
    get_const_function__GetAvailableHomingMethods_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<basicmicro_ros2::srv::GetAvailableHomingMethods_Request *>(untyped_value);
  value = item;
}

void assign_function__GetAvailableHomingMethods_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<basicmicro_ros2::srv::GetAvailableHomingMethods_Request *>(
    get_function__GetAvailableHomingMethods_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const basicmicro_ros2::srv::GetAvailableHomingMethods_Request *>(untyped_value);
  item = value;
}

void resize_function__GetAvailableHomingMethods_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<basicmicro_ros2::srv::GetAvailableHomingMethods_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetAvailableHomingMethods_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<basicmicro_ros2::srv::GetAvailableHomingMethods_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetAvailableHomingMethods_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<basicmicro_ros2::srv::GetAvailableHomingMethods_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__GetAvailableHomingMethods_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<basicmicro_ros2::srv::GetAvailableHomingMethods_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetAvailableHomingMethods_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const basicmicro_ros2::srv::GetAvailableHomingMethods_Response *>(
    get_const_function__GetAvailableHomingMethods_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<basicmicro_ros2::srv::GetAvailableHomingMethods_Response *>(untyped_value);
  value = item;
}

void assign_function__GetAvailableHomingMethods_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<basicmicro_ros2::srv::GetAvailableHomingMethods_Response *>(
    get_function__GetAvailableHomingMethods_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const basicmicro_ros2::srv::GetAvailableHomingMethods_Response *>(untyped_value);
  item = value;
}

void resize_function__GetAvailableHomingMethods_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<basicmicro_ros2::srv::GetAvailableHomingMethods_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetAvailableHomingMethods_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetAvailableHomingMethods_Event__request,  // size() function pointer
    get_const_function__GetAvailableHomingMethods_Event__request,  // get_const(index) function pointer
    get_function__GetAvailableHomingMethods_Event__request,  // get(index) function pointer
    fetch_function__GetAvailableHomingMethods_Event__request,  // fetch(index, &value) function pointer
    assign_function__GetAvailableHomingMethods_Event__request,  // assign(index, value) function pointer
    resize_function__GetAvailableHomingMethods_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(basicmicro_ros2::srv::GetAvailableHomingMethods_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetAvailableHomingMethods_Event__response,  // size() function pointer
    get_const_function__GetAvailableHomingMethods_Event__response,  // get_const(index) function pointer
    get_function__GetAvailableHomingMethods_Event__response,  // get(index) function pointer
    fetch_function__GetAvailableHomingMethods_Event__response,  // fetch(index, &value) function pointer
    assign_function__GetAvailableHomingMethods_Event__response,  // assign(index, value) function pointer
    resize_function__GetAvailableHomingMethods_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetAvailableHomingMethods_Event_message_members = {
  "basicmicro_ros2::srv",  // message namespace
  "GetAvailableHomingMethods_Event",  // message name
  3,  // number of fields
  sizeof(basicmicro_ros2::srv::GetAvailableHomingMethods_Event),
  false,  // has_any_key_member_
  GetAvailableHomingMethods_Event_message_member_array,  // message members
  GetAvailableHomingMethods_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  GetAvailableHomingMethods_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetAvailableHomingMethods_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetAvailableHomingMethods_Event_message_members,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace basicmicro_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Event>()
{
  return &::basicmicro_ros2::srv::rosidl_typesupport_introspection_cpp::GetAvailableHomingMethods_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods_Event)() {
  return &::basicmicro_ros2::srv::rosidl_typesupport_introspection_cpp::GetAvailableHomingMethods_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace basicmicro_ros2
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GetAvailableHomingMethods_service_members = {
  "basicmicro_ros2::srv",  // service namespace
  "GetAvailableHomingMethods",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t GetAvailableHomingMethods_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetAvailableHomingMethods_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<basicmicro_ros2::srv::GetAvailableHomingMethods>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<basicmicro_ros2::srv::GetAvailableHomingMethods>,
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace basicmicro_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::basicmicro_ros2::srv::rosidl_typesupport_introspection_cpp::GetAvailableHomingMethods_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::basicmicro_ros2::srv::GetAvailableHomingMethods_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::basicmicro_ros2::srv::GetAvailableHomingMethods_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::basicmicro_ros2::srv::GetAvailableHomingMethods_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, basicmicro_ros2, srv, GetAvailableHomingMethods)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<basicmicro_ros2::srv::GetAvailableHomingMethods>();
}

#ifdef __cplusplus
}
#endif
