// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/MoveToAbsolutePosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/move_to_absolute_position.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__MOVE_TO_ABSOLUTE_POSITION__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__MOVE_TO_ABSOLUTE_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/move_to_absolute_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_MoveToAbsolutePosition_Request_buffer_command
{
public:
  explicit Init_MoveToAbsolutePosition_Request_buffer_command(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Request buffer_command(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request::_buffer_command_type arg)
  {
    msg_.buffer_command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Request msg_;
};

class Init_MoveToAbsolutePosition_Request_deceleration
{
public:
  explicit Init_MoveToAbsolutePosition_Request_deceleration(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request & msg)
  : msg_(msg)
  {}
  Init_MoveToAbsolutePosition_Request_buffer_command deceleration(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request::_deceleration_type arg)
  {
    msg_.deceleration = std::move(arg);
    return Init_MoveToAbsolutePosition_Request_buffer_command(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Request msg_;
};

class Init_MoveToAbsolutePosition_Request_acceleration
{
public:
  explicit Init_MoveToAbsolutePosition_Request_acceleration(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request & msg)
  : msg_(msg)
  {}
  Init_MoveToAbsolutePosition_Request_deceleration acceleration(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_MoveToAbsolutePosition_Request_deceleration(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Request msg_;
};

class Init_MoveToAbsolutePosition_Request_max_speed
{
public:
  explicit Init_MoveToAbsolutePosition_Request_max_speed(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request & msg)
  : msg_(msg)
  {}
  Init_MoveToAbsolutePosition_Request_acceleration max_speed(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request::_max_speed_type arg)
  {
    msg_.max_speed = std::move(arg);
    return Init_MoveToAbsolutePosition_Request_acceleration(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Request msg_;
};

class Init_MoveToAbsolutePosition_Request_right_position_radians
{
public:
  explicit Init_MoveToAbsolutePosition_Request_right_position_radians(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request & msg)
  : msg_(msg)
  {}
  Init_MoveToAbsolutePosition_Request_max_speed right_position_radians(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request::_right_position_radians_type arg)
  {
    msg_.right_position_radians = std::move(arg);
    return Init_MoveToAbsolutePosition_Request_max_speed(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Request msg_;
};

class Init_MoveToAbsolutePosition_Request_left_position_radians
{
public:
  Init_MoveToAbsolutePosition_Request_left_position_radians()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToAbsolutePosition_Request_right_position_radians left_position_radians(::basicmicro_ros2::srv::MoveToAbsolutePosition_Request::_left_position_radians_type arg)
  {
    msg_.left_position_radians = std::move(arg);
    return Init_MoveToAbsolutePosition_Request_right_position_radians(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::MoveToAbsolutePosition_Request>()
{
  return basicmicro_ros2::srv::builder::Init_MoveToAbsolutePosition_Request_left_position_radians();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_MoveToAbsolutePosition_Response_message
{
public:
  explicit Init_MoveToAbsolutePosition_Response_message(::basicmicro_ros2::srv::MoveToAbsolutePosition_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Response message(::basicmicro_ros2::srv::MoveToAbsolutePosition_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Response msg_;
};

class Init_MoveToAbsolutePosition_Response_success
{
public:
  Init_MoveToAbsolutePosition_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToAbsolutePosition_Response_message success(::basicmicro_ros2::srv::MoveToAbsolutePosition_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MoveToAbsolutePosition_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::MoveToAbsolutePosition_Response>()
{
  return basicmicro_ros2::srv::builder::Init_MoveToAbsolutePosition_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_MoveToAbsolutePosition_Event_response
{
public:
  explicit Init_MoveToAbsolutePosition_Event_response(::basicmicro_ros2::srv::MoveToAbsolutePosition_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Event response(::basicmicro_ros2::srv::MoveToAbsolutePosition_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Event msg_;
};

class Init_MoveToAbsolutePosition_Event_request
{
public:
  explicit Init_MoveToAbsolutePosition_Event_request(::basicmicro_ros2::srv::MoveToAbsolutePosition_Event & msg)
  : msg_(msg)
  {}
  Init_MoveToAbsolutePosition_Event_response request(::basicmicro_ros2::srv::MoveToAbsolutePosition_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveToAbsolutePosition_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Event msg_;
};

class Init_MoveToAbsolutePosition_Event_info
{
public:
  Init_MoveToAbsolutePosition_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToAbsolutePosition_Event_request info(::basicmicro_ros2::srv::MoveToAbsolutePosition_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveToAbsolutePosition_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveToAbsolutePosition_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::MoveToAbsolutePosition_Event>()
{
  return basicmicro_ros2::srv::builder::Init_MoveToAbsolutePosition_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__MOVE_TO_ABSOLUTE_POSITION__BUILDER_HPP_
