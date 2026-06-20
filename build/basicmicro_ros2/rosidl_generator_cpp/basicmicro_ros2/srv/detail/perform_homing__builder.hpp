// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/PerformHoming.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/perform_homing.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__PERFORM_HOMING__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__PERFORM_HOMING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/perform_homing__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_PerformHoming_Request_timeout
{
public:
  explicit Init_PerformHoming_Request_timeout(::basicmicro_ros2::srv::PerformHoming_Request & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::PerformHoming_Request timeout(::basicmicro_ros2::srv::PerformHoming_Request::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::PerformHoming_Request msg_;
};

class Init_PerformHoming_Request_homing_speed
{
public:
  explicit Init_PerformHoming_Request_homing_speed(::basicmicro_ros2::srv::PerformHoming_Request & msg)
  : msg_(msg)
  {}
  Init_PerformHoming_Request_timeout homing_speed(::basicmicro_ros2::srv::PerformHoming_Request::_homing_speed_type arg)
  {
    msg_.homing_speed = std::move(arg);
    return Init_PerformHoming_Request_timeout(msg_);
  }

private:
  ::basicmicro_ros2::srv::PerformHoming_Request msg_;
};

class Init_PerformHoming_Request_direction
{
public:
  explicit Init_PerformHoming_Request_direction(::basicmicro_ros2::srv::PerformHoming_Request & msg)
  : msg_(msg)
  {}
  Init_PerformHoming_Request_homing_speed direction(::basicmicro_ros2::srv::PerformHoming_Request::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_PerformHoming_Request_homing_speed(msg_);
  }

private:
  ::basicmicro_ros2::srv::PerformHoming_Request msg_;
};

class Init_PerformHoming_Request_method_id
{
public:
  Init_PerformHoming_Request_method_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformHoming_Request_direction method_id(::basicmicro_ros2::srv::PerformHoming_Request::_method_id_type arg)
  {
    msg_.method_id = std::move(arg);
    return Init_PerformHoming_Request_direction(msg_);
  }

private:
  ::basicmicro_ros2::srv::PerformHoming_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::PerformHoming_Request>()
{
  return basicmicro_ros2::srv::builder::Init_PerformHoming_Request_method_id();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_PerformHoming_Response_encoder_zeroed
{
public:
  explicit Init_PerformHoming_Response_encoder_zeroed(::basicmicro_ros2::srv::PerformHoming_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::PerformHoming_Response encoder_zeroed(::basicmicro_ros2::srv::PerformHoming_Response::_encoder_zeroed_type arg)
  {
    msg_.encoder_zeroed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::PerformHoming_Response msg_;
};

class Init_PerformHoming_Response_message
{
public:
  explicit Init_PerformHoming_Response_message(::basicmicro_ros2::srv::PerformHoming_Response & msg)
  : msg_(msg)
  {}
  Init_PerformHoming_Response_encoder_zeroed message(::basicmicro_ros2::srv::PerformHoming_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_PerformHoming_Response_encoder_zeroed(msg_);
  }

private:
  ::basicmicro_ros2::srv::PerformHoming_Response msg_;
};

class Init_PerformHoming_Response_success
{
public:
  Init_PerformHoming_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformHoming_Response_message success(::basicmicro_ros2::srv::PerformHoming_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PerformHoming_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::PerformHoming_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::PerformHoming_Response>()
{
  return basicmicro_ros2::srv::builder::Init_PerformHoming_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_PerformHoming_Event_response
{
public:
  explicit Init_PerformHoming_Event_response(::basicmicro_ros2::srv::PerformHoming_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::PerformHoming_Event response(::basicmicro_ros2::srv::PerformHoming_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::PerformHoming_Event msg_;
};

class Init_PerformHoming_Event_request
{
public:
  explicit Init_PerformHoming_Event_request(::basicmicro_ros2::srv::PerformHoming_Event & msg)
  : msg_(msg)
  {}
  Init_PerformHoming_Event_response request(::basicmicro_ros2::srv::PerformHoming_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PerformHoming_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::PerformHoming_Event msg_;
};

class Init_PerformHoming_Event_info
{
public:
  Init_PerformHoming_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PerformHoming_Event_request info(::basicmicro_ros2::srv::PerformHoming_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PerformHoming_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::PerformHoming_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::PerformHoming_Event>()
{
  return basicmicro_ros2::srv::builder::Init_PerformHoming_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__PERFORM_HOMING__BUILDER_HPP_
