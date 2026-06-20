// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/GetAvailableHomingMethods.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/get_available_homing_methods.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__GET_AVAILABLE_HOMING_METHODS__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__GET_AVAILABLE_HOMING_METHODS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::GetAvailableHomingMethods_Request>()
{
  return ::basicmicro_ros2::srv::GetAvailableHomingMethods_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_GetAvailableHomingMethods_Response_acts_as_limit
{
public:
  explicit Init_GetAvailableHomingMethods_Response_acts_as_limit(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Response acts_as_limit(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response::_acts_as_limit_type arg)
  {
    msg_.acts_as_limit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Response msg_;
};

class Init_GetAvailableHomingMethods_Response_auto_zeros_encoder
{
public:
  explicit Init_GetAvailableHomingMethods_Response_auto_zeros_encoder(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response & msg)
  : msg_(msg)
  {}
  Init_GetAvailableHomingMethods_Response_acts_as_limit auto_zeros_encoder(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response::_auto_zeros_encoder_type arg)
  {
    msg_.auto_zeros_encoder = std::move(arg);
    return Init_GetAvailableHomingMethods_Response_acts_as_limit(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Response msg_;
};

class Init_GetAvailableHomingMethods_Response_allowed_directions
{
public:
  explicit Init_GetAvailableHomingMethods_Response_allowed_directions(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response & msg)
  : msg_(msg)
  {}
  Init_GetAvailableHomingMethods_Response_auto_zeros_encoder allowed_directions(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response::_allowed_directions_type arg)
  {
    msg_.allowed_directions = std::move(arg);
    return Init_GetAvailableHomingMethods_Response_auto_zeros_encoder(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Response msg_;
};

class Init_GetAvailableHomingMethods_Response_method_descriptions
{
public:
  explicit Init_GetAvailableHomingMethods_Response_method_descriptions(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response & msg)
  : msg_(msg)
  {}
  Init_GetAvailableHomingMethods_Response_allowed_directions method_descriptions(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response::_method_descriptions_type arg)
  {
    msg_.method_descriptions = std::move(arg);
    return Init_GetAvailableHomingMethods_Response_allowed_directions(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Response msg_;
};

class Init_GetAvailableHomingMethods_Response_available_methods
{
public:
  explicit Init_GetAvailableHomingMethods_Response_available_methods(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response & msg)
  : msg_(msg)
  {}
  Init_GetAvailableHomingMethods_Response_method_descriptions available_methods(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response::_available_methods_type arg)
  {
    msg_.available_methods = std::move(arg);
    return Init_GetAvailableHomingMethods_Response_method_descriptions(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Response msg_;
};

class Init_GetAvailableHomingMethods_Response_controller_type
{
public:
  explicit Init_GetAvailableHomingMethods_Response_controller_type(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response & msg)
  : msg_(msg)
  {}
  Init_GetAvailableHomingMethods_Response_available_methods controller_type(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response::_controller_type_type arg)
  {
    msg_.controller_type = std::move(arg);
    return Init_GetAvailableHomingMethods_Response_available_methods(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Response msg_;
};

class Init_GetAvailableHomingMethods_Response_success
{
public:
  Init_GetAvailableHomingMethods_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetAvailableHomingMethods_Response_controller_type success(::basicmicro_ros2::srv::GetAvailableHomingMethods_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetAvailableHomingMethods_Response_controller_type(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::GetAvailableHomingMethods_Response>()
{
  return basicmicro_ros2::srv::builder::Init_GetAvailableHomingMethods_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_GetAvailableHomingMethods_Event_response
{
public:
  explicit Init_GetAvailableHomingMethods_Event_response(::basicmicro_ros2::srv::GetAvailableHomingMethods_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Event response(::basicmicro_ros2::srv::GetAvailableHomingMethods_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Event msg_;
};

class Init_GetAvailableHomingMethods_Event_request
{
public:
  explicit Init_GetAvailableHomingMethods_Event_request(::basicmicro_ros2::srv::GetAvailableHomingMethods_Event & msg)
  : msg_(msg)
  {}
  Init_GetAvailableHomingMethods_Event_response request(::basicmicro_ros2::srv::GetAvailableHomingMethods_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetAvailableHomingMethods_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Event msg_;
};

class Init_GetAvailableHomingMethods_Event_info
{
public:
  Init_GetAvailableHomingMethods_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetAvailableHomingMethods_Event_request info(::basicmicro_ros2::srv::GetAvailableHomingMethods_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetAvailableHomingMethods_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetAvailableHomingMethods_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::GetAvailableHomingMethods_Event>()
{
  return basicmicro_ros2::srv::builder::Init_GetAvailableHomingMethods_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__GET_AVAILABLE_HOMING_METHODS__BUILDER_HPP_
