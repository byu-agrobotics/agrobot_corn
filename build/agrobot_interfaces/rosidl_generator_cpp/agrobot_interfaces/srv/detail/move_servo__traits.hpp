// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from agrobot_interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "agrobot_interfaces/srv/move_servo.hpp"


#ifndef AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__TRAITS_HPP_
#define AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "agrobot_interfaces/srv/detail/move_servo__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace agrobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveServo_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: request
  {
    out << "request: ";
    rosidl_generator_traits::value_to_yaml(msg.request, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveServo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request: ";
    rosidl_generator_traits::value_to_yaml(msg.request, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveServo_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace agrobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use agrobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const agrobot_interfaces::srv::MoveServo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  agrobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use agrobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const agrobot_interfaces::srv::MoveServo_Request & msg)
{
  return agrobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<agrobot_interfaces::srv::MoveServo_Request>()
{
  return "agrobot_interfaces::srv::MoveServo_Request";
}

template<>
inline const char * name<agrobot_interfaces::srv::MoveServo_Request>()
{
  return "agrobot_interfaces/srv/MoveServo_Request";
}

template<>
struct has_fixed_size<agrobot_interfaces::srv::MoveServo_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<agrobot_interfaces::srv::MoveServo_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<agrobot_interfaces::srv::MoveServo_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace agrobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveServo_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveServo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveServo_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace agrobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use agrobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const agrobot_interfaces::srv::MoveServo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  agrobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use agrobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const agrobot_interfaces::srv::MoveServo_Response & msg)
{
  return agrobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<agrobot_interfaces::srv::MoveServo_Response>()
{
  return "agrobot_interfaces::srv::MoveServo_Response";
}

template<>
inline const char * name<agrobot_interfaces::srv::MoveServo_Response>()
{
  return "agrobot_interfaces/srv/MoveServo_Response";
}

template<>
struct has_fixed_size<agrobot_interfaces::srv::MoveServo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<agrobot_interfaces::srv::MoveServo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<agrobot_interfaces::srv::MoveServo_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace agrobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveServo_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveServo_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveServo_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace agrobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use agrobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const agrobot_interfaces::srv::MoveServo_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  agrobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use agrobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const agrobot_interfaces::srv::MoveServo_Event & msg)
{
  return agrobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<agrobot_interfaces::srv::MoveServo_Event>()
{
  return "agrobot_interfaces::srv::MoveServo_Event";
}

template<>
inline const char * name<agrobot_interfaces::srv::MoveServo_Event>()
{
  return "agrobot_interfaces/srv/MoveServo_Event";
}

template<>
struct has_fixed_size<agrobot_interfaces::srv::MoveServo_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<agrobot_interfaces::srv::MoveServo_Event>
  : std::integral_constant<bool, has_bounded_size<agrobot_interfaces::srv::MoveServo_Request>::value && has_bounded_size<agrobot_interfaces::srv::MoveServo_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<agrobot_interfaces::srv::MoveServo_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<agrobot_interfaces::srv::MoveServo>()
{
  return "agrobot_interfaces::srv::MoveServo";
}

template<>
inline const char * name<agrobot_interfaces::srv::MoveServo>()
{
  return "agrobot_interfaces/srv/MoveServo";
}

template<>
struct has_fixed_size<agrobot_interfaces::srv::MoveServo>
  : std::integral_constant<
    bool,
    has_fixed_size<agrobot_interfaces::srv::MoveServo_Request>::value &&
    has_fixed_size<agrobot_interfaces::srv::MoveServo_Response>::value
  >
{
};

template<>
struct has_bounded_size<agrobot_interfaces::srv::MoveServo>
  : std::integral_constant<
    bool,
    has_bounded_size<agrobot_interfaces::srv::MoveServo_Request>::value &&
    has_bounded_size<agrobot_interfaces::srv::MoveServo_Response>::value
  >
{
};

template<>
struct is_service<agrobot_interfaces::srv::MoveServo>
  : std::true_type
{
};

template<>
struct is_service_request<agrobot_interfaces::srv::MoveServo_Request>
  : std::true_type
{
};

template<>
struct is_service_response<agrobot_interfaces::srv::MoveServo_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__TRAITS_HPP_
