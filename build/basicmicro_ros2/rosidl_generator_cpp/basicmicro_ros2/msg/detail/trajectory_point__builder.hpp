// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:msg/TrajectoryPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/msg/trajectory_point.hpp"


#ifndef BASICMICRO_ROS2__MSG__DETAIL__TRAJECTORY_POINT__BUILDER_HPP_
#define BASICMICRO_ROS2__MSG__DETAIL__TRAJECTORY_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/msg/detail/trajectory_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace msg
{

namespace builder
{

class Init_TrajectoryPoint_duration
{
public:
  explicit Init_TrajectoryPoint_duration(::basicmicro_ros2::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::msg::TrajectoryPoint duration(::basicmicro_ros2::msg::TrajectoryPoint::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_acceleration
{
public:
  explicit Init_TrajectoryPoint_acceleration(::basicmicro_ros2::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPoint_duration acceleration(::basicmicro_ros2::msg::TrajectoryPoint::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_TrajectoryPoint_duration(msg_);
  }

private:
  ::basicmicro_ros2::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_speed
{
public:
  explicit Init_TrajectoryPoint_speed(::basicmicro_ros2::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPoint_acceleration speed(::basicmicro_ros2::msg::TrajectoryPoint::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_TrajectoryPoint_acceleration(msg_);
  }

private:
  ::basicmicro_ros2::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_deceleration
{
public:
  explicit Init_TrajectoryPoint_deceleration(::basicmicro_ros2::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPoint_speed deceleration(::basicmicro_ros2::msg::TrajectoryPoint::_deceleration_type arg)
  {
    msg_.deceleration = std::move(arg);
    return Init_TrajectoryPoint_speed(msg_);
  }

private:
  ::basicmicro_ros2::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_right_position
{
public:
  explicit Init_TrajectoryPoint_right_position(::basicmicro_ros2::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPoint_deceleration right_position(::basicmicro_ros2::msg::TrajectoryPoint::_right_position_type arg)
  {
    msg_.right_position = std::move(arg);
    return Init_TrajectoryPoint_deceleration(msg_);
  }

private:
  ::basicmicro_ros2::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_left_position
{
public:
  explicit Init_TrajectoryPoint_left_position(::basicmicro_ros2::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPoint_right_position left_position(::basicmicro_ros2::msg::TrajectoryPoint::_left_position_type arg)
  {
    msg_.left_position = std::move(arg);
    return Init_TrajectoryPoint_right_position(msg_);
  }

private:
  ::basicmicro_ros2::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_right_distance
{
public:
  explicit Init_TrajectoryPoint_right_distance(::basicmicro_ros2::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPoint_left_position right_distance(::basicmicro_ros2::msg::TrajectoryPoint::_right_distance_type arg)
  {
    msg_.right_distance = std::move(arg);
    return Init_TrajectoryPoint_left_position(msg_);
  }

private:
  ::basicmicro_ros2::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_left_distance
{
public:
  explicit Init_TrajectoryPoint_left_distance(::basicmicro_ros2::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPoint_right_distance left_distance(::basicmicro_ros2::msg::TrajectoryPoint::_left_distance_type arg)
  {
    msg_.left_distance = std::move(arg);
    return Init_TrajectoryPoint_right_distance(msg_);
  }

private:
  ::basicmicro_ros2::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_command_type
{
public:
  Init_TrajectoryPoint_command_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajectoryPoint_left_distance command_type(::basicmicro_ros2::msg::TrajectoryPoint::_command_type_type arg)
  {
    msg_.command_type = std::move(arg);
    return Init_TrajectoryPoint_left_distance(msg_);
  }

private:
  ::basicmicro_ros2::msg::TrajectoryPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::msg::TrajectoryPoint>()
{
  return basicmicro_ros2::msg::builder::Init_TrajectoryPoint_command_type();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__MSG__DETAIL__TRAJECTORY_POINT__BUILDER_HPP_
