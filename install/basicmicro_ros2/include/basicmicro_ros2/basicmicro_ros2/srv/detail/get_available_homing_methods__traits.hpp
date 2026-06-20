// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from basicmicro_ros2:srv/GetAvailableHomingMethods.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/get_available_homing_methods.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__GET_AVAILABLE_HOMING_METHODS__TRAITS_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__GET_AVAILABLE_HOMING_METHODS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace basicmicro_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAvailableHomingMethods_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetAvailableHomingMethods_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetAvailableHomingMethods_Request & msg, bool use_flow_style = false)
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

}  // namespace basicmicro_ros2

namespace rosidl_generator_traits
{

[[deprecated("use basicmicro_ros2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const basicmicro_ros2::srv::GetAvailableHomingMethods_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  basicmicro_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use basicmicro_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const basicmicro_ros2::srv::GetAvailableHomingMethods_Request & msg)
{
  return basicmicro_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>()
{
  return "basicmicro_ros2::srv::GetAvailableHomingMethods_Request";
}

template<>
inline const char * name<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>()
{
  return "basicmicro_ros2/srv/GetAvailableHomingMethods_Request";
}

template<>
struct has_fixed_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace basicmicro_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAvailableHomingMethods_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: controller_type
  {
    out << "controller_type: ";
    rosidl_generator_traits::value_to_yaml(msg.controller_type, out);
    out << ", ";
  }

  // member: available_methods
  {
    if (msg.available_methods.size() == 0) {
      out << "available_methods: []";
    } else {
      out << "available_methods: [";
      size_t pending_items = msg.available_methods.size();
      for (auto item : msg.available_methods) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: method_descriptions
  {
    if (msg.method_descriptions.size() == 0) {
      out << "method_descriptions: []";
    } else {
      out << "method_descriptions: [";
      size_t pending_items = msg.method_descriptions.size();
      for (auto item : msg.method_descriptions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: allowed_directions
  {
    if (msg.allowed_directions.size() == 0) {
      out << "allowed_directions: []";
    } else {
      out << "allowed_directions: [";
      size_t pending_items = msg.allowed_directions.size();
      for (auto item : msg.allowed_directions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: auto_zeros_encoder
  {
    if (msg.auto_zeros_encoder.size() == 0) {
      out << "auto_zeros_encoder: []";
    } else {
      out << "auto_zeros_encoder: [";
      size_t pending_items = msg.auto_zeros_encoder.size();
      for (auto item : msg.auto_zeros_encoder) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: acts_as_limit
  {
    if (msg.acts_as_limit.size() == 0) {
      out << "acts_as_limit: []";
    } else {
      out << "acts_as_limit: [";
      size_t pending_items = msg.acts_as_limit.size();
      for (auto item : msg.acts_as_limit) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const GetAvailableHomingMethods_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: controller_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "controller_type: ";
    rosidl_generator_traits::value_to_yaml(msg.controller_type, out);
    out << "\n";
  }

  // member: available_methods
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.available_methods.size() == 0) {
      out << "available_methods: []\n";
    } else {
      out << "available_methods:\n";
      for (auto item : msg.available_methods) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: method_descriptions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.method_descriptions.size() == 0) {
      out << "method_descriptions: []\n";
    } else {
      out << "method_descriptions:\n";
      for (auto item : msg.method_descriptions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: allowed_directions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.allowed_directions.size() == 0) {
      out << "allowed_directions: []\n";
    } else {
      out << "allowed_directions:\n";
      for (auto item : msg.allowed_directions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: auto_zeros_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.auto_zeros_encoder.size() == 0) {
      out << "auto_zeros_encoder: []\n";
    } else {
      out << "auto_zeros_encoder:\n";
      for (auto item : msg.auto_zeros_encoder) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: acts_as_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.acts_as_limit.size() == 0) {
      out << "acts_as_limit: []\n";
    } else {
      out << "acts_as_limit:\n";
      for (auto item : msg.acts_as_limit) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetAvailableHomingMethods_Response & msg, bool use_flow_style = false)
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

}  // namespace basicmicro_ros2

namespace rosidl_generator_traits
{

[[deprecated("use basicmicro_ros2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const basicmicro_ros2::srv::GetAvailableHomingMethods_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  basicmicro_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use basicmicro_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const basicmicro_ros2::srv::GetAvailableHomingMethods_Response & msg)
{
  return basicmicro_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>()
{
  return "basicmicro_ros2::srv::GetAvailableHomingMethods_Response";
}

template<>
inline const char * name<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>()
{
  return "basicmicro_ros2/srv/GetAvailableHomingMethods_Response";
}

template<>
struct has_fixed_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace basicmicro_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetAvailableHomingMethods_Event & msg,
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
  const GetAvailableHomingMethods_Event & msg,
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

inline std::string to_yaml(const GetAvailableHomingMethods_Event & msg, bool use_flow_style = false)
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

}  // namespace basicmicro_ros2

namespace rosidl_generator_traits
{

[[deprecated("use basicmicro_ros2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const basicmicro_ros2::srv::GetAvailableHomingMethods_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  basicmicro_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use basicmicro_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const basicmicro_ros2::srv::GetAvailableHomingMethods_Event & msg)
{
  return basicmicro_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<basicmicro_ros2::srv::GetAvailableHomingMethods_Event>()
{
  return "basicmicro_ros2::srv::GetAvailableHomingMethods_Event";
}

template<>
inline const char * name<basicmicro_ros2::srv::GetAvailableHomingMethods_Event>()
{
  return "basicmicro_ros2/srv/GetAvailableHomingMethods_Event";
}

template<>
struct has_fixed_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Event>
  : std::integral_constant<bool, has_bounded_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>::value && has_bounded_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<basicmicro_ros2::srv::GetAvailableHomingMethods_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<basicmicro_ros2::srv::GetAvailableHomingMethods>()
{
  return "basicmicro_ros2::srv::GetAvailableHomingMethods";
}

template<>
inline const char * name<basicmicro_ros2::srv::GetAvailableHomingMethods>()
{
  return "basicmicro_ros2/srv/GetAvailableHomingMethods";
}

template<>
struct has_fixed_size<basicmicro_ros2::srv::GetAvailableHomingMethods>
  : std::integral_constant<
    bool,
    has_fixed_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>::value &&
    has_fixed_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>::value
  >
{
};

template<>
struct has_bounded_size<basicmicro_ros2::srv::GetAvailableHomingMethods>
  : std::integral_constant<
    bool,
    has_bounded_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>::value &&
    has_bounded_size<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>::value
  >
{
};

template<>
struct is_service<basicmicro_ros2::srv::GetAvailableHomingMethods>
  : std::true_type
{
};

template<>
struct is_service_request<basicmicro_ros2::srv::GetAvailableHomingMethods_Request>
  : std::true_type
{
};

template<>
struct is_service_response<basicmicro_ros2::srv::GetAvailableHomingMethods_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BASICMICRO_ROS2__SRV__DETAIL__GET_AVAILABLE_HOMING_METHODS__TRAITS_HPP_
