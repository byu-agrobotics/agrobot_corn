// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/SetPositionLimits.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/set_position_limits.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__SET_POSITION_LIMITS__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__SET_POSITION_LIMITS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/set_position_limits__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetPositionLimits_Request_decel_rate
{
public:
  explicit Init_SetPositionLimits_Request_decel_rate(::basicmicro_ros2::srv::SetPositionLimits_Request & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetPositionLimits_Request decel_rate(::basicmicro_ros2::srv::SetPositionLimits_Request::_decel_rate_type arg)
  {
    msg_.decel_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Request msg_;
};

class Init_SetPositionLimits_Request_violation_behavior
{
public:
  explicit Init_SetPositionLimits_Request_violation_behavior(::basicmicro_ros2::srv::SetPositionLimits_Request & msg)
  : msg_(msg)
  {}
  Init_SetPositionLimits_Request_decel_rate violation_behavior(::basicmicro_ros2::srv::SetPositionLimits_Request::_violation_behavior_type arg)
  {
    msg_.violation_behavior = std::move(arg);
    return Init_SetPositionLimits_Request_decel_rate(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Request msg_;
};

class Init_SetPositionLimits_Request_right_max_position
{
public:
  explicit Init_SetPositionLimits_Request_right_max_position(::basicmicro_ros2::srv::SetPositionLimits_Request & msg)
  : msg_(msg)
  {}
  Init_SetPositionLimits_Request_violation_behavior right_max_position(::basicmicro_ros2::srv::SetPositionLimits_Request::_right_max_position_type arg)
  {
    msg_.right_max_position = std::move(arg);
    return Init_SetPositionLimits_Request_violation_behavior(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Request msg_;
};

class Init_SetPositionLimits_Request_right_min_position
{
public:
  explicit Init_SetPositionLimits_Request_right_min_position(::basicmicro_ros2::srv::SetPositionLimits_Request & msg)
  : msg_(msg)
  {}
  Init_SetPositionLimits_Request_right_max_position right_min_position(::basicmicro_ros2::srv::SetPositionLimits_Request::_right_min_position_type arg)
  {
    msg_.right_min_position = std::move(arg);
    return Init_SetPositionLimits_Request_right_max_position(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Request msg_;
};

class Init_SetPositionLimits_Request_left_max_position
{
public:
  explicit Init_SetPositionLimits_Request_left_max_position(::basicmicro_ros2::srv::SetPositionLimits_Request & msg)
  : msg_(msg)
  {}
  Init_SetPositionLimits_Request_right_min_position left_max_position(::basicmicro_ros2::srv::SetPositionLimits_Request::_left_max_position_type arg)
  {
    msg_.left_max_position = std::move(arg);
    return Init_SetPositionLimits_Request_right_min_position(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Request msg_;
};

class Init_SetPositionLimits_Request_left_min_position
{
public:
  explicit Init_SetPositionLimits_Request_left_min_position(::basicmicro_ros2::srv::SetPositionLimits_Request & msg)
  : msg_(msg)
  {}
  Init_SetPositionLimits_Request_left_max_position left_min_position(::basicmicro_ros2::srv::SetPositionLimits_Request::_left_min_position_type arg)
  {
    msg_.left_min_position = std::move(arg);
    return Init_SetPositionLimits_Request_left_max_position(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Request msg_;
};

class Init_SetPositionLimits_Request_enable_limits
{
public:
  Init_SetPositionLimits_Request_enable_limits()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPositionLimits_Request_left_min_position enable_limits(::basicmicro_ros2::srv::SetPositionLimits_Request::_enable_limits_type arg)
  {
    msg_.enable_limits = std::move(arg);
    return Init_SetPositionLimits_Request_left_min_position(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetPositionLimits_Request>()
{
  return basicmicro_ros2::srv::builder::Init_SetPositionLimits_Request_enable_limits();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetPositionLimits_Response_message
{
public:
  explicit Init_SetPositionLimits_Response_message(::basicmicro_ros2::srv::SetPositionLimits_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetPositionLimits_Response message(::basicmicro_ros2::srv::SetPositionLimits_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Response msg_;
};

class Init_SetPositionLimits_Response_success
{
public:
  Init_SetPositionLimits_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPositionLimits_Response_message success(::basicmicro_ros2::srv::SetPositionLimits_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetPositionLimits_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetPositionLimits_Response>()
{
  return basicmicro_ros2::srv::builder::Init_SetPositionLimits_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetPositionLimits_Event_response
{
public:
  explicit Init_SetPositionLimits_Event_response(::basicmicro_ros2::srv::SetPositionLimits_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetPositionLimits_Event response(::basicmicro_ros2::srv::SetPositionLimits_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Event msg_;
};

class Init_SetPositionLimits_Event_request
{
public:
  explicit Init_SetPositionLimits_Event_request(::basicmicro_ros2::srv::SetPositionLimits_Event & msg)
  : msg_(msg)
  {}
  Init_SetPositionLimits_Event_response request(::basicmicro_ros2::srv::SetPositionLimits_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetPositionLimits_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Event msg_;
};

class Init_SetPositionLimits_Event_info
{
public:
  Init_SetPositionLimits_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPositionLimits_Event_request info(::basicmicro_ros2::srv::SetPositionLimits_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetPositionLimits_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetPositionLimits_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetPositionLimits_Event>()
{
  return basicmicro_ros2::srv::builder::Init_SetPositionLimits_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__SET_POSITION_LIMITS__BUILDER_HPP_
