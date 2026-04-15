// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from agrobot_interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "agrobot_interfaces/srv/move_servo.hpp"


#ifndef AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__BUILDER_HPP_
#define AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "agrobot_interfaces/srv/detail/move_servo__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace agrobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveServo_Request_request
{
public:
  Init_MoveServo_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::agrobot_interfaces::srv::MoveServo_Request request(::agrobot_interfaces::srv::MoveServo_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::agrobot_interfaces::srv::MoveServo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::agrobot_interfaces::srv::MoveServo_Request>()
{
  return agrobot_interfaces::srv::builder::Init_MoveServo_Request_request();
}

}  // namespace agrobot_interfaces


namespace agrobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveServo_Response_response
{
public:
  Init_MoveServo_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::agrobot_interfaces::srv::MoveServo_Response response(::agrobot_interfaces::srv::MoveServo_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::agrobot_interfaces::srv::MoveServo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::agrobot_interfaces::srv::MoveServo_Response>()
{
  return agrobot_interfaces::srv::builder::Init_MoveServo_Response_response();
}

}  // namespace agrobot_interfaces


namespace agrobot_interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveServo_Event_response
{
public:
  explicit Init_MoveServo_Event_response(::agrobot_interfaces::srv::MoveServo_Event & msg)
  : msg_(msg)
  {}
  ::agrobot_interfaces::srv::MoveServo_Event response(::agrobot_interfaces::srv::MoveServo_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::agrobot_interfaces::srv::MoveServo_Event msg_;
};

class Init_MoveServo_Event_request
{
public:
  explicit Init_MoveServo_Event_request(::agrobot_interfaces::srv::MoveServo_Event & msg)
  : msg_(msg)
  {}
  Init_MoveServo_Event_response request(::agrobot_interfaces::srv::MoveServo_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveServo_Event_response(msg_);
  }

private:
  ::agrobot_interfaces::srv::MoveServo_Event msg_;
};

class Init_MoveServo_Event_info
{
public:
  Init_MoveServo_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveServo_Event_request info(::agrobot_interfaces::srv::MoveServo_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveServo_Event_request(msg_);
  }

private:
  ::agrobot_interfaces::srv::MoveServo_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::agrobot_interfaces::srv::MoveServo_Event>()
{
  return agrobot_interfaces::srv::builder::Init_MoveServo_Event_info();
}

}  // namespace agrobot_interfaces

#endif  // AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__BUILDER_HPP_
