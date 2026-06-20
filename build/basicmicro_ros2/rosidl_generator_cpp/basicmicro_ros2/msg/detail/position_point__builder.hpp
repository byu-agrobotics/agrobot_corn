// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:msg/PositionPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/msg/position_point.hpp"


#ifndef BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__BUILDER_HPP_
#define BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/msg/detail/position_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace msg
{

namespace builder
{

class Init_PositionPoint_deceleration
{
public:
  explicit Init_PositionPoint_deceleration(::basicmicro_ros2::msg::PositionPoint & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::msg::PositionPoint deceleration(::basicmicro_ros2::msg::PositionPoint::_deceleration_type arg)
  {
    msg_.deceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::msg::PositionPoint msg_;
};

class Init_PositionPoint_acceleration
{
public:
  explicit Init_PositionPoint_acceleration(::basicmicro_ros2::msg::PositionPoint & msg)
  : msg_(msg)
  {}
  Init_PositionPoint_deceleration acceleration(::basicmicro_ros2::msg::PositionPoint::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_PositionPoint_deceleration(msg_);
  }

private:
  ::basicmicro_ros2::msg::PositionPoint msg_;
};

class Init_PositionPoint_max_speed
{
public:
  explicit Init_PositionPoint_max_speed(::basicmicro_ros2::msg::PositionPoint & msg)
  : msg_(msg)
  {}
  Init_PositionPoint_acceleration max_speed(::basicmicro_ros2::msg::PositionPoint::_max_speed_type arg)
  {
    msg_.max_speed = std::move(arg);
    return Init_PositionPoint_acceleration(msg_);
  }

private:
  ::basicmicro_ros2::msg::PositionPoint msg_;
};

class Init_PositionPoint_right_position
{
public:
  explicit Init_PositionPoint_right_position(::basicmicro_ros2::msg::PositionPoint & msg)
  : msg_(msg)
  {}
  Init_PositionPoint_max_speed right_position(::basicmicro_ros2::msg::PositionPoint::_right_position_type arg)
  {
    msg_.right_position = std::move(arg);
    return Init_PositionPoint_max_speed(msg_);
  }

private:
  ::basicmicro_ros2::msg::PositionPoint msg_;
};

class Init_PositionPoint_left_position
{
public:
  Init_PositionPoint_left_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionPoint_right_position left_position(::basicmicro_ros2::msg::PositionPoint::_left_position_type arg)
  {
    msg_.left_position = std::move(arg);
    return Init_PositionPoint_right_position(msg_);
  }

private:
  ::basicmicro_ros2::msg::PositionPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::msg::PositionPoint>()
{
  return basicmicro_ros2::msg::builder::Init_PositionPoint_left_position();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__BUILDER_HPP_
