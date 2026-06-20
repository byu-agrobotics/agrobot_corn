// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from basicmicro_ros2:srv/ExecuteTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/execute_trajectory.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_TRAJECTORY__TRAITS_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "basicmicro_ros2/srv/detail/execute_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'trajectory_points'
#include "basicmicro_ros2/msg/detail/trajectory_point__traits.hpp"

namespace basicmicro_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const ExecuteTrajectory_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: trajectory_points
  {
    if (msg.trajectory_points.size() == 0) {
      out << "trajectory_points: []";
    } else {
      out << "trajectory_points: [";
      size_t pending_items = msg.trajectory_points.size();
      for (auto item : msg.trajectory_points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: trajectory_type
  {
    out << "trajectory_type: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteTrajectory_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: trajectory_points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.trajectory_points.size() == 0) {
      out << "trajectory_points: []\n";
    } else {
      out << "trajectory_points:\n";
      for (auto item : msg.trajectory_points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: trajectory_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trajectory_type: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteTrajectory_Request & msg, bool use_flow_style = false)
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
  const basicmicro_ros2::srv::ExecuteTrajectory_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  basicmicro_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use basicmicro_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const basicmicro_ros2::srv::ExecuteTrajectory_Request & msg)
{
  return basicmicro_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<basicmicro_ros2::srv::ExecuteTrajectory_Request>()
{
  return "basicmicro_ros2::srv::ExecuteTrajectory_Request";
}

template<>
inline const char * name<basicmicro_ros2::srv::ExecuteTrajectory_Request>()
{
  return "basicmicro_ros2/srv/ExecuteTrajectory_Request";
}

template<>
struct has_fixed_size<basicmicro_ros2::srv::ExecuteTrajectory_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<basicmicro_ros2::srv::ExecuteTrajectory_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<basicmicro_ros2::srv::ExecuteTrajectory_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace basicmicro_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const ExecuteTrajectory_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: total_commands_sent
  {
    out << "total_commands_sent: ";
    rosidl_generator_traits::value_to_yaml(msg.total_commands_sent, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteTrajectory_Response & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: total_commands_sent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_commands_sent: ";
    rosidl_generator_traits::value_to_yaml(msg.total_commands_sent, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteTrajectory_Response & msg, bool use_flow_style = false)
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
  const basicmicro_ros2::srv::ExecuteTrajectory_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  basicmicro_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use basicmicro_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const basicmicro_ros2::srv::ExecuteTrajectory_Response & msg)
{
  return basicmicro_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<basicmicro_ros2::srv::ExecuteTrajectory_Response>()
{
  return "basicmicro_ros2::srv::ExecuteTrajectory_Response";
}

template<>
inline const char * name<basicmicro_ros2::srv::ExecuteTrajectory_Response>()
{
  return "basicmicro_ros2/srv/ExecuteTrajectory_Response";
}

template<>
struct has_fixed_size<basicmicro_ros2::srv::ExecuteTrajectory_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<basicmicro_ros2::srv::ExecuteTrajectory_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<basicmicro_ros2::srv::ExecuteTrajectory_Response>
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
  const ExecuteTrajectory_Event & msg,
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
  const ExecuteTrajectory_Event & msg,
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

inline std::string to_yaml(const ExecuteTrajectory_Event & msg, bool use_flow_style = false)
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
  const basicmicro_ros2::srv::ExecuteTrajectory_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  basicmicro_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use basicmicro_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const basicmicro_ros2::srv::ExecuteTrajectory_Event & msg)
{
  return basicmicro_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<basicmicro_ros2::srv::ExecuteTrajectory_Event>()
{
  return "basicmicro_ros2::srv::ExecuteTrajectory_Event";
}

template<>
inline const char * name<basicmicro_ros2::srv::ExecuteTrajectory_Event>()
{
  return "basicmicro_ros2/srv/ExecuteTrajectory_Event";
}

template<>
struct has_fixed_size<basicmicro_ros2::srv::ExecuteTrajectory_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<basicmicro_ros2::srv::ExecuteTrajectory_Event>
  : std::integral_constant<bool, has_bounded_size<basicmicro_ros2::srv::ExecuteTrajectory_Request>::value && has_bounded_size<basicmicro_ros2::srv::ExecuteTrajectory_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<basicmicro_ros2::srv::ExecuteTrajectory_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<basicmicro_ros2::srv::ExecuteTrajectory>()
{
  return "basicmicro_ros2::srv::ExecuteTrajectory";
}

template<>
inline const char * name<basicmicro_ros2::srv::ExecuteTrajectory>()
{
  return "basicmicro_ros2/srv/ExecuteTrajectory";
}

template<>
struct has_fixed_size<basicmicro_ros2::srv::ExecuteTrajectory>
  : std::integral_constant<
    bool,
    has_fixed_size<basicmicro_ros2::srv::ExecuteTrajectory_Request>::value &&
    has_fixed_size<basicmicro_ros2::srv::ExecuteTrajectory_Response>::value
  >
{
};

template<>
struct has_bounded_size<basicmicro_ros2::srv::ExecuteTrajectory>
  : std::integral_constant<
    bool,
    has_bounded_size<basicmicro_ros2::srv::ExecuteTrajectory_Request>::value &&
    has_bounded_size<basicmicro_ros2::srv::ExecuteTrajectory_Response>::value
  >
{
};

template<>
struct is_service<basicmicro_ros2::srv::ExecuteTrajectory>
  : std::true_type
{
};

template<>
struct is_service_request<basicmicro_ros2::srv::ExecuteTrajectory_Request>
  : std::true_type
{
};

template<>
struct is_service_response<basicmicro_ros2::srv::ExecuteTrajectory_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_TRAJECTORY__TRAITS_HPP_
