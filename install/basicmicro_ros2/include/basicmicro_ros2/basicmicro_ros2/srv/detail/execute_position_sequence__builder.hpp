// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/ExecutePositionSequence.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/execute_position_sequence.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_POSITION_SEQUENCE__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_POSITION_SEQUENCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/execute_position_sequence__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_ExecutePositionSequence_Request_position_points
{
public:
  Init_ExecutePositionSequence_Request_position_points()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::basicmicro_ros2::srv::ExecutePositionSequence_Request position_points(::basicmicro_ros2::srv::ExecutePositionSequence_Request::_position_points_type arg)
  {
    msg_.position_points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecutePositionSequence_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::ExecutePositionSequence_Request>()
{
  return basicmicro_ros2::srv::builder::Init_ExecutePositionSequence_Request_position_points();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_ExecutePositionSequence_Response_total_commands_sent
{
public:
  explicit Init_ExecutePositionSequence_Response_total_commands_sent(::basicmicro_ros2::srv::ExecutePositionSequence_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::ExecutePositionSequence_Response total_commands_sent(::basicmicro_ros2::srv::ExecutePositionSequence_Response::_total_commands_sent_type arg)
  {
    msg_.total_commands_sent = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecutePositionSequence_Response msg_;
};

class Init_ExecutePositionSequence_Response_message
{
public:
  explicit Init_ExecutePositionSequence_Response_message(::basicmicro_ros2::srv::ExecutePositionSequence_Response & msg)
  : msg_(msg)
  {}
  Init_ExecutePositionSequence_Response_total_commands_sent message(::basicmicro_ros2::srv::ExecutePositionSequence_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ExecutePositionSequence_Response_total_commands_sent(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecutePositionSequence_Response msg_;
};

class Init_ExecutePositionSequence_Response_success
{
public:
  Init_ExecutePositionSequence_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecutePositionSequence_Response_message success(::basicmicro_ros2::srv::ExecutePositionSequence_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ExecutePositionSequence_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecutePositionSequence_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::ExecutePositionSequence_Response>()
{
  return basicmicro_ros2::srv::builder::Init_ExecutePositionSequence_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_ExecutePositionSequence_Event_response
{
public:
  explicit Init_ExecutePositionSequence_Event_response(::basicmicro_ros2::srv::ExecutePositionSequence_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::ExecutePositionSequence_Event response(::basicmicro_ros2::srv::ExecutePositionSequence_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecutePositionSequence_Event msg_;
};

class Init_ExecutePositionSequence_Event_request
{
public:
  explicit Init_ExecutePositionSequence_Event_request(::basicmicro_ros2::srv::ExecutePositionSequence_Event & msg)
  : msg_(msg)
  {}
  Init_ExecutePositionSequence_Event_response request(::basicmicro_ros2::srv::ExecutePositionSequence_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ExecutePositionSequence_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecutePositionSequence_Event msg_;
};

class Init_ExecutePositionSequence_Event_info
{
public:
  Init_ExecutePositionSequence_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecutePositionSequence_Event_request info(::basicmicro_ros2::srv::ExecutePositionSequence_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ExecutePositionSequence_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecutePositionSequence_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::ExecutePositionSequence_Event>()
{
  return basicmicro_ros2::srv::builder::Init_ExecutePositionSequence_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_POSITION_SEQUENCE__BUILDER_HPP_
