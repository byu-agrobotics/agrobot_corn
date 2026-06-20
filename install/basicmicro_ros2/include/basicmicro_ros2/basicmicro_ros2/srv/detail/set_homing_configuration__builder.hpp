// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/SetHomingConfiguration.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/set_homing_configuration.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__SET_HOMING_CONFIGURATION__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__SET_HOMING_CONFIGURATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/set_homing_configuration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetHomingConfiguration_Request_default_homing_speed
{
public:
  explicit Init_SetHomingConfiguration_Request_default_homing_speed(::basicmicro_ros2::srv::SetHomingConfiguration_Request & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetHomingConfiguration_Request default_homing_speed(::basicmicro_ros2::srv::SetHomingConfiguration_Request::_default_homing_speed_type arg)
  {
    msg_.default_homing_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetHomingConfiguration_Request msg_;
};

class Init_SetHomingConfiguration_Request_default_homing_method
{
public:
  explicit Init_SetHomingConfiguration_Request_default_homing_method(::basicmicro_ros2::srv::SetHomingConfiguration_Request & msg)
  : msg_(msg)
  {}
  Init_SetHomingConfiguration_Request_default_homing_speed default_homing_method(::basicmicro_ros2::srv::SetHomingConfiguration_Request::_default_homing_method_type arg)
  {
    msg_.default_homing_method = std::move(arg);
    return Init_SetHomingConfiguration_Request_default_homing_speed(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetHomingConfiguration_Request msg_;
};

class Init_SetHomingConfiguration_Request_auto_home_on_startup
{
public:
  Init_SetHomingConfiguration_Request_auto_home_on_startup()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetHomingConfiguration_Request_default_homing_method auto_home_on_startup(::basicmicro_ros2::srv::SetHomingConfiguration_Request::_auto_home_on_startup_type arg)
  {
    msg_.auto_home_on_startup = std::move(arg);
    return Init_SetHomingConfiguration_Request_default_homing_method(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetHomingConfiguration_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetHomingConfiguration_Request>()
{
  return basicmicro_ros2::srv::builder::Init_SetHomingConfiguration_Request_auto_home_on_startup();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetHomingConfiguration_Response_message
{
public:
  explicit Init_SetHomingConfiguration_Response_message(::basicmicro_ros2::srv::SetHomingConfiguration_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetHomingConfiguration_Response message(::basicmicro_ros2::srv::SetHomingConfiguration_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetHomingConfiguration_Response msg_;
};

class Init_SetHomingConfiguration_Response_success
{
public:
  Init_SetHomingConfiguration_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetHomingConfiguration_Response_message success(::basicmicro_ros2::srv::SetHomingConfiguration_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetHomingConfiguration_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetHomingConfiguration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetHomingConfiguration_Response>()
{
  return basicmicro_ros2::srv::builder::Init_SetHomingConfiguration_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetHomingConfiguration_Event_response
{
public:
  explicit Init_SetHomingConfiguration_Event_response(::basicmicro_ros2::srv::SetHomingConfiguration_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetHomingConfiguration_Event response(::basicmicro_ros2::srv::SetHomingConfiguration_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetHomingConfiguration_Event msg_;
};

class Init_SetHomingConfiguration_Event_request
{
public:
  explicit Init_SetHomingConfiguration_Event_request(::basicmicro_ros2::srv::SetHomingConfiguration_Event & msg)
  : msg_(msg)
  {}
  Init_SetHomingConfiguration_Event_response request(::basicmicro_ros2::srv::SetHomingConfiguration_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetHomingConfiguration_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetHomingConfiguration_Event msg_;
};

class Init_SetHomingConfiguration_Event_info
{
public:
  Init_SetHomingConfiguration_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetHomingConfiguration_Event_request info(::basicmicro_ros2::srv::SetHomingConfiguration_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetHomingConfiguration_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetHomingConfiguration_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetHomingConfiguration_Event>()
{
  return basicmicro_ros2::srv::builder::Init_SetHomingConfiguration_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__SET_HOMING_CONFIGURATION__BUILDER_HPP_
