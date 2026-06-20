// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/SetMotionStrategy.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/set_motion_strategy.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__SET_MOTION_STRATEGY__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__SET_MOTION_STRATEGY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/set_motion_strategy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetMotionStrategy_Request_strategy
{
public:
  Init_SetMotionStrategy_Request_strategy()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::basicmicro_ros2::srv::SetMotionStrategy_Request strategy(::basicmicro_ros2::srv::SetMotionStrategy_Request::_strategy_type arg)
  {
    msg_.strategy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionStrategy_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetMotionStrategy_Request>()
{
  return basicmicro_ros2::srv::builder::Init_SetMotionStrategy_Request_strategy();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetMotionStrategy_Response_message
{
public:
  explicit Init_SetMotionStrategy_Response_message(::basicmicro_ros2::srv::SetMotionStrategy_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetMotionStrategy_Response message(::basicmicro_ros2::srv::SetMotionStrategy_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionStrategy_Response msg_;
};

class Init_SetMotionStrategy_Response_success
{
public:
  Init_SetMotionStrategy_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMotionStrategy_Response_message success(::basicmicro_ros2::srv::SetMotionStrategy_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetMotionStrategy_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionStrategy_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetMotionStrategy_Response>()
{
  return basicmicro_ros2::srv::builder::Init_SetMotionStrategy_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_SetMotionStrategy_Event_response
{
public:
  explicit Init_SetMotionStrategy_Event_response(::basicmicro_ros2::srv::SetMotionStrategy_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::SetMotionStrategy_Event response(::basicmicro_ros2::srv::SetMotionStrategy_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionStrategy_Event msg_;
};

class Init_SetMotionStrategy_Event_request
{
public:
  explicit Init_SetMotionStrategy_Event_request(::basicmicro_ros2::srv::SetMotionStrategy_Event & msg)
  : msg_(msg)
  {}
  Init_SetMotionStrategy_Event_response request(::basicmicro_ros2::srv::SetMotionStrategy_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetMotionStrategy_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionStrategy_Event msg_;
};

class Init_SetMotionStrategy_Event_info
{
public:
  Init_SetMotionStrategy_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMotionStrategy_Event_request info(::basicmicro_ros2::srv::SetMotionStrategy_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetMotionStrategy_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::SetMotionStrategy_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::SetMotionStrategy_Event>()
{
  return basicmicro_ros2::srv::builder::Init_SetMotionStrategy_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__SET_MOTION_STRATEGY__BUILDER_HPP_
