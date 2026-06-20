// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/MoveDistance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/move_distance.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__MOVE_DISTANCE__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__MOVE_DISTANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/move_distance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_MoveDistance_Request_use_buffer
{
public:
  explicit Init_MoveDistance_Request_use_buffer(::basicmicro_ros2::srv::MoveDistance_Request & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::MoveDistance_Request use_buffer(::basicmicro_ros2::srv::MoveDistance_Request::_use_buffer_type arg)
  {
    msg_.use_buffer = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Request msg_;
};

class Init_MoveDistance_Request_acceleration
{
public:
  explicit Init_MoveDistance_Request_acceleration(::basicmicro_ros2::srv::MoveDistance_Request & msg)
  : msg_(msg)
  {}
  Init_MoveDistance_Request_use_buffer acceleration(::basicmicro_ros2::srv::MoveDistance_Request::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_MoveDistance_Request_use_buffer(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Request msg_;
};

class Init_MoveDistance_Request_speed
{
public:
  explicit Init_MoveDistance_Request_speed(::basicmicro_ros2::srv::MoveDistance_Request & msg)
  : msg_(msg)
  {}
  Init_MoveDistance_Request_acceleration speed(::basicmicro_ros2::srv::MoveDistance_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_MoveDistance_Request_acceleration(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Request msg_;
};

class Init_MoveDistance_Request_right_distance
{
public:
  explicit Init_MoveDistance_Request_right_distance(::basicmicro_ros2::srv::MoveDistance_Request & msg)
  : msg_(msg)
  {}
  Init_MoveDistance_Request_speed right_distance(::basicmicro_ros2::srv::MoveDistance_Request::_right_distance_type arg)
  {
    msg_.right_distance = std::move(arg);
    return Init_MoveDistance_Request_speed(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Request msg_;
};

class Init_MoveDistance_Request_left_distance
{
public:
  Init_MoveDistance_Request_left_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveDistance_Request_right_distance left_distance(::basicmicro_ros2::srv::MoveDistance_Request::_left_distance_type arg)
  {
    msg_.left_distance = std::move(arg);
    return Init_MoveDistance_Request_right_distance(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::MoveDistance_Request>()
{
  return basicmicro_ros2::srv::builder::Init_MoveDistance_Request_left_distance();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_MoveDistance_Response_buffer_slots_used
{
public:
  explicit Init_MoveDistance_Response_buffer_slots_used(::basicmicro_ros2::srv::MoveDistance_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::MoveDistance_Response buffer_slots_used(::basicmicro_ros2::srv::MoveDistance_Response::_buffer_slots_used_type arg)
  {
    msg_.buffer_slots_used = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Response msg_;
};

class Init_MoveDistance_Response_message
{
public:
  explicit Init_MoveDistance_Response_message(::basicmicro_ros2::srv::MoveDistance_Response & msg)
  : msg_(msg)
  {}
  Init_MoveDistance_Response_buffer_slots_used message(::basicmicro_ros2::srv::MoveDistance_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_MoveDistance_Response_buffer_slots_used(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Response msg_;
};

class Init_MoveDistance_Response_success
{
public:
  Init_MoveDistance_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveDistance_Response_message success(::basicmicro_ros2::srv::MoveDistance_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MoveDistance_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::MoveDistance_Response>()
{
  return basicmicro_ros2::srv::builder::Init_MoveDistance_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_MoveDistance_Event_response
{
public:
  explicit Init_MoveDistance_Event_response(::basicmicro_ros2::srv::MoveDistance_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::MoveDistance_Event response(::basicmicro_ros2::srv::MoveDistance_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Event msg_;
};

class Init_MoveDistance_Event_request
{
public:
  explicit Init_MoveDistance_Event_request(::basicmicro_ros2::srv::MoveDistance_Event & msg)
  : msg_(msg)
  {}
  Init_MoveDistance_Event_response request(::basicmicro_ros2::srv::MoveDistance_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveDistance_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Event msg_;
};

class Init_MoveDistance_Event_info
{
public:
  Init_MoveDistance_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveDistance_Event_request info(::basicmicro_ros2::srv::MoveDistance_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveDistance_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::MoveDistance_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::MoveDistance_Event>()
{
  return basicmicro_ros2::srv::builder::Init_MoveDistance_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__MOVE_DISTANCE__BUILDER_HPP_
