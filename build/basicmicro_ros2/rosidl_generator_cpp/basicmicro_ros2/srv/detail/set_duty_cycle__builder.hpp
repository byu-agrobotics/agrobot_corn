// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/SetDutyCycle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/set_duty_cycle.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__SET_DUTY_CYCLE__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__SET_DUTY_CYCLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/set_duty_cycle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetDutyCycle_Request_acceleration
{
public:
  explicit Init_SetDutyCycle_Request_acceleration(::basicmicro_ros2::srv::SetDutyCycle_Request & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetDutyCycle_Request acceleration(::basicmicro_ros2::srv::SetDutyCycle_Request::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetDutyCycle_Request msg_;
};

class Init_SetDutyCycle_Request_use_acceleration
{
public:
  explicit Init_SetDutyCycle_Request_use_acceleration(::basicmicro_ros2::srv::SetDutyCycle_Request & msg)
  : msg_(msg)
  {}
  Init_SetDutyCycle_Request_acceleration use_acceleration(::basicmicro_ros2::srv::SetDutyCycle_Request::_use_acceleration_type arg)
  {
    msg_.use_acceleration = std::move(arg);
    return Init_SetDutyCycle_Request_acceleration(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetDutyCycle_Request msg_;
};

class Init_SetDutyCycle_Request_right_duty
{
public:
  explicit Init_SetDutyCycle_Request_right_duty(::basicmicro_ros2::srv::SetDutyCycle_Request & msg)
  : msg_(msg)
  {}
  Init_SetDutyCycle_Request_use_acceleration right_duty(::basicmicro_ros2::srv::SetDutyCycle_Request::_right_duty_type arg)
  {
    msg_.right_duty = std::move(arg);
    return Init_SetDutyCycle_Request_use_acceleration(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetDutyCycle_Request msg_;
};

class Init_SetDutyCycle_Request_left_duty
{
public:
  Init_SetDutyCycle_Request_left_duty()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetDutyCycle_Request_right_duty left_duty(::basicmicro_ros2::srv::SetDutyCycle_Request::_left_duty_type arg)
  {
    msg_.left_duty = std::move(arg);
    return Init_SetDutyCycle_Request_right_duty(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetDutyCycle_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetDutyCycle_Request>()
{
  return basicmicro_ros2::srv::builder::Init_SetDutyCycle_Request_left_duty();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetDutyCycle_Response_message
{
public:
  explicit Init_SetDutyCycle_Response_message(::basicmicro_ros2::srv::SetDutyCycle_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetDutyCycle_Response message(::basicmicro_ros2::srv::SetDutyCycle_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetDutyCycle_Response msg_;
};

class Init_SetDutyCycle_Response_success
{
public:
  Init_SetDutyCycle_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetDutyCycle_Response_message success(::basicmicro_ros2::srv::SetDutyCycle_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetDutyCycle_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetDutyCycle_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetDutyCycle_Response>()
{
  return basicmicro_ros2::srv::builder::Init_SetDutyCycle_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetDutyCycle_Event_response
{
public:
  explicit Init_SetDutyCycle_Event_response(::basicmicro_ros2::srv::SetDutyCycle_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetDutyCycle_Event response(::basicmicro_ros2::srv::SetDutyCycle_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetDutyCycle_Event msg_;
};

class Init_SetDutyCycle_Event_request
{
public:
  explicit Init_SetDutyCycle_Event_request(::basicmicro_ros2::srv::SetDutyCycle_Event & msg)
  : msg_(msg)
  {}
  Init_SetDutyCycle_Event_response request(::basicmicro_ros2::srv::SetDutyCycle_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetDutyCycle_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetDutyCycle_Event msg_;
};

class Init_SetDutyCycle_Event_info
{
public:
  Init_SetDutyCycle_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetDutyCycle_Event_request info(::basicmicro_ros2::srv::SetDutyCycle_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetDutyCycle_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetDutyCycle_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetDutyCycle_Event>()
{
  return basicmicro_ros2::srv::builder::Init_SetDutyCycle_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__SET_DUTY_CYCLE__BUILDER_HPP_
