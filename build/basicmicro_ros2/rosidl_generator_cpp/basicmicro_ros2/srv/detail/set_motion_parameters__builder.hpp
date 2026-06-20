// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/SetMotionParameters.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/set_motion_parameters.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__SET_MOTION_PARAMETERS__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__SET_MOTION_PARAMETERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/set_motion_parameters__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetMotionParameters_Request_buffer_depth
{
public:
  explicit Init_SetMotionParameters_Request_buffer_depth(::basicmicro_ros2::srv::SetMotionParameters_Request & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetMotionParameters_Request buffer_depth(::basicmicro_ros2::srv::SetMotionParameters_Request::_buffer_depth_type arg)
  {
    msg_.buffer_depth = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionParameters_Request msg_;
};

class Init_SetMotionParameters_Request_max_speed
{
public:
  explicit Init_SetMotionParameters_Request_max_speed(::basicmicro_ros2::srv::SetMotionParameters_Request & msg)
  : msg_(msg)
  {}
  Init_SetMotionParameters_Request_buffer_depth max_speed(::basicmicro_ros2::srv::SetMotionParameters_Request::_max_speed_type arg)
  {
    msg_.max_speed = std::move(arg);
    return Init_SetMotionParameters_Request_buffer_depth(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionParameters_Request msg_;
};

class Init_SetMotionParameters_Request_default_acceleration
{
public:
  Init_SetMotionParameters_Request_default_acceleration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMotionParameters_Request_max_speed default_acceleration(::basicmicro_ros2::srv::SetMotionParameters_Request::_default_acceleration_type arg)
  {
    msg_.default_acceleration = std::move(arg);
    return Init_SetMotionParameters_Request_max_speed(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionParameters_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetMotionParameters_Request>()
{
  return basicmicro_ros2::srv::builder::Init_SetMotionParameters_Request_default_acceleration();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetMotionParameters_Response_message
{
public:
  explicit Init_SetMotionParameters_Response_message(::basicmicro_ros2::srv::SetMotionParameters_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetMotionParameters_Response message(::basicmicro_ros2::srv::SetMotionParameters_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionParameters_Response msg_;
};

class Init_SetMotionParameters_Response_success
{
public:
  Init_SetMotionParameters_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMotionParameters_Response_message success(::basicmicro_ros2::srv::SetMotionParameters_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetMotionParameters_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionParameters_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetMotionParameters_Response>()
{
  return basicmicro_ros2::srv::builder::Init_SetMotionParameters_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetMotionParameters_Event_response
{
public:
  explicit Init_SetMotionParameters_Event_response(::basicmicro_ros2::srv::SetMotionParameters_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetMotionParameters_Event response(::basicmicro_ros2::srv::SetMotionParameters_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionParameters_Event msg_;
};

class Init_SetMotionParameters_Event_request
{
public:
  explicit Init_SetMotionParameters_Event_request(::basicmicro_ros2::srv::SetMotionParameters_Event & msg)
  : msg_(msg)
  {}
  Init_SetMotionParameters_Event_response request(::basicmicro_ros2::srv::SetMotionParameters_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetMotionParameters_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionParameters_Event msg_;
};

class Init_SetMotionParameters_Event_info
{
public:
  Init_SetMotionParameters_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMotionParameters_Event_request info(::basicmicro_ros2::srv::SetMotionParameters_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetMotionParameters_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionParameters_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetMotionParameters_Event>()
{
  return basicmicro_ros2::srv::builder::Init_SetMotionParameters_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__SET_MOTION_PARAMETERS__BUILDER_HPP_
