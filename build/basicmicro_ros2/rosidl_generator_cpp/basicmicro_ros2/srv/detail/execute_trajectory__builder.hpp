// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/ExecuteTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/execute_trajectory.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_TRAJECTORY__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/execute_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_ExecuteTrajectory_Request_trajectory_type
{
public:
  explicit Init_ExecuteTrajectory_Request_trajectory_type(::basicmicro_ros2::srv::ExecuteTrajectory_Request & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::ExecuteTrajectory_Request trajectory_type(::basicmicro_ros2::srv::ExecuteTrajectory_Request::_trajectory_type_type arg)
  {
    msg_.trajectory_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecuteTrajectory_Request msg_;
};

class Init_ExecuteTrajectory_Request_trajectory_points
{
public:
  Init_ExecuteTrajectory_Request_trajectory_points()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteTrajectory_Request_trajectory_type trajectory_points(::basicmicro_ros2::srv::ExecuteTrajectory_Request::_trajectory_points_type arg)
  {
    msg_.trajectory_points = std::move(arg);
    return Init_ExecuteTrajectory_Request_trajectory_type(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecuteTrajectory_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::ExecuteTrajectory_Request>()
{
  return basicmicro_ros2::srv::builder::Init_ExecuteTrajectory_Request_trajectory_points();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_ExecuteTrajectory_Response_total_commands_sent
{
public:
  explicit Init_ExecuteTrajectory_Response_total_commands_sent(::basicmicro_ros2::srv::ExecuteTrajectory_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::ExecuteTrajectory_Response total_commands_sent(::basicmicro_ros2::srv::ExecuteTrajectory_Response::_total_commands_sent_type arg)
  {
    msg_.total_commands_sent = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecuteTrajectory_Response msg_;
};

class Init_ExecuteTrajectory_Response_message
{
public:
  explicit Init_ExecuteTrajectory_Response_message(::basicmicro_ros2::srv::ExecuteTrajectory_Response & msg)
  : msg_(msg)
  {}
  Init_ExecuteTrajectory_Response_total_commands_sent message(::basicmicro_ros2::srv::ExecuteTrajectory_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ExecuteTrajectory_Response_total_commands_sent(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecuteTrajectory_Response msg_;
};

class Init_ExecuteTrajectory_Response_success
{
public:
  Init_ExecuteTrajectory_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteTrajectory_Response_message success(::basicmicro_ros2::srv::ExecuteTrajectory_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ExecuteTrajectory_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecuteTrajectory_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::ExecuteTrajectory_Response>()
{
  return basicmicro_ros2::srv::builder::Init_ExecuteTrajectory_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_ExecuteTrajectory_Event_response
{
public:
  explicit Init_ExecuteTrajectory_Event_response(::basicmicro_ros2::srv::ExecuteTrajectory_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::ExecuteTrajectory_Event response(::basicmicro_ros2::srv::ExecuteTrajectory_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecuteTrajectory_Event msg_;
};

class Init_ExecuteTrajectory_Event_request
{
public:
  explicit Init_ExecuteTrajectory_Event_request(::basicmicro_ros2::srv::ExecuteTrajectory_Event & msg)
  : msg_(msg)
  {}
  Init_ExecuteTrajectory_Event_response request(::basicmicro_ros2::srv::ExecuteTrajectory_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ExecuteTrajectory_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecuteTrajectory_Event msg_;
};

class Init_ExecuteTrajectory_Event_info
{
public:
  Init_ExecuteTrajectory_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteTrajectory_Event_request info(::basicmicro_ros2::srv::ExecuteTrajectory_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ExecuteTrajectory_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::ExecuteTrajectory_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::ExecuteTrajectory_Event>()
{
  return basicmicro_ros2::srv::builder::Init_ExecuteTrajectory_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_TRAJECTORY__BUILDER_HPP_
