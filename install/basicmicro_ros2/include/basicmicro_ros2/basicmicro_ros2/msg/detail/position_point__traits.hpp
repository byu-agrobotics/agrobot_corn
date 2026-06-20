// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from basicmicro_ros2:msg/PositionPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/msg/position_point.hpp"


#ifndef BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__TRAITS_HPP_
#define BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "basicmicro_ros2/msg/detail/position_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace basicmicro_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const PositionPoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_position
  {
    out << "left_position: ";
    rosidl_generator_traits::value_to_yaml(msg.left_position, out);
    out << ", ";
  }

  // member: right_position
  {
    out << "right_position: ";
    rosidl_generator_traits::value_to_yaml(msg.right_position, out);
    out << ", ";
  }

  // member: max_speed
  {
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
    out << ", ";
  }

  // member: acceleration
  {
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << ", ";
  }

  // member: deceleration
  {
    out << "deceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.deceleration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PositionPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_position: ";
    rosidl_generator_traits::value_to_yaml(msg.left_position, out);
    out << "\n";
  }

  // member: right_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_position: ";
    rosidl_generator_traits::value_to_yaml(msg.right_position, out);
    out << "\n";
  }

  // member: max_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
    out << "\n";
  }

  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << "\n";
  }

  // member: deceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "deceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.deceleration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PositionPoint & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace basicmicro_ros2

namespace rosidl_generator_traits
{

[[deprecated("use basicmicro_ros2::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const basicmicro_ros2::msg::PositionPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  basicmicro_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use basicmicro_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const basicmicro_ros2::msg::PositionPoint & msg)
{
  return basicmicro_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<basicmicro_ros2::msg::PositionPoint>()
{
  return "basicmicro_ros2::msg::PositionPoint";
}

template<>
inline const char * name<basicmicro_ros2::msg::PositionPoint>()
{
  return "basicmicro_ros2/msg/PositionPoint";
}

template<>
struct has_fixed_size<basicmicro_ros2::msg::PositionPoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<basicmicro_ros2::msg::PositionPoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<basicmicro_ros2::msg::PositionPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__TRAITS_HPP_
