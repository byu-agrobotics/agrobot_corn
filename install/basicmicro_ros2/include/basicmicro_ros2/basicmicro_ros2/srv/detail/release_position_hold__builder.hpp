// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/ReleasePositionHold.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/release_position_hold.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__RELEASE_POSITION_HOLD__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__RELEASE_POSITION_HOLD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/release_position_hold__struct.hpp"
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
auto build<::basicmicro_ros2::srv::ReleasePositionHold_Request>()
{
  return ::basicmicro_ros2::srv::ReleasePositionHold_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_ReleasePositionHold_Response_message
{
public:
  explicit Init_ReleasePositionHold_Response_message(::basicmicro_ros2::srv::ReleasePositionHold_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::ReleasePositionHold_Response message(::basicmicro_ros2::srv::ReleasePositionHold_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::ReleasePositionHold_Response msg_;
};

class Init_ReleasePositionHold_Response_success
{
public:
  Init_ReleasePositionHold_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReleasePositionHold_Response_message success(::basicmicro_ros2::srv::ReleasePositionHold_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ReleasePositionHold_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::ReleasePositionHold_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::ReleasePositionHold_Response>()
{
  return basicmicro_ros2::srv::builder::Init_ReleasePositionHold_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_ReleasePositionHold_Event_response
{
public:
  explicit Init_ReleasePositionHold_Event_response(::basicmicro_ros2::srv::ReleasePositionHold_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::ReleasePositionHold_Event response(::basicmicro_ros2::srv::ReleasePositionHold_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::ReleasePositionHold_Event msg_;
};

class Init_ReleasePositionHold_Event_request
{
public:
  explicit Init_ReleasePositionHold_Event_request(::basicmicro_ros2::srv::ReleasePositionHold_Event & msg)
  : msg_(msg)
  {}
  Init_ReleasePositionHold_Event_response request(::basicmicro_ros2::srv::ReleasePositionHold_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ReleasePositionHold_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::ReleasePositionHold_Event msg_;
};

class Init_ReleasePositionHold_Event_info
{
public:
  Init_ReleasePositionHold_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReleasePositionHold_Event_request info(::basicmicro_ros2::srv::ReleasePositionHold_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ReleasePositionHold_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::ReleasePositionHold_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::ReleasePositionHold_Event>()
{
  return basicmicro_ros2::srv::builder::Init_ReleasePositionHold_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__RELEASE_POSITION_HOLD__BUILDER_HPP_
