// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from basicmicro_ros2:srv/GetServoStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/get_servo_status.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__GET_SERVO_STATUS__BUILDER_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__GET_SERVO_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "basicmicro_ros2/srv/detail/get_servo_status__struct.hpp"
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
auto build<::basicmicro_ros2::srv::GetServoStatus_Request>()
{
  return ::basicmicro_ros2::srv::GetServoStatus_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_GetServoStatus_Response_message
{
public:
  explicit Init_GetServoStatus_Response_message(::basicmicro_ros2::srv::GetServoStatus_Response & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::GetServoStatus_Response message(::basicmicro_ros2::srv::GetServoStatus_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetServoStatus_Response msg_;
};

class Init_GetServoStatus_Response_error_limits_exceeded
{
public:
  explicit Init_GetServoStatus_Response_error_limits_exceeded(::basicmicro_ros2::srv::GetServoStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetServoStatus_Response_message error_limits_exceeded(::basicmicro_ros2::srv::GetServoStatus_Response::_error_limits_exceeded_type arg)
  {
    msg_.error_limits_exceeded = std::move(arg);
    return Init_GetServoStatus_Response_message(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetServoStatus_Response msg_;
};

class Init_GetServoStatus_Response_right_speed_error
{
public:
  explicit Init_GetServoStatus_Response_right_speed_error(::basicmicro_ros2::srv::GetServoStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetServoStatus_Response_error_limits_exceeded right_speed_error(::basicmicro_ros2::srv::GetServoStatus_Response::_right_speed_error_type arg)
  {
    msg_.right_speed_error = std::move(arg);
    return Init_GetServoStatus_Response_error_limits_exceeded(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetServoStatus_Response msg_;
};

class Init_GetServoStatus_Response_left_speed_error
{
public:
  explicit Init_GetServoStatus_Response_left_speed_error(::basicmicro_ros2::srv::GetServoStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetServoStatus_Response_right_speed_error left_speed_error(::basicmicro_ros2::srv::GetServoStatus_Response::_left_speed_error_type arg)
  {
    msg_.left_speed_error = std::move(arg);
    return Init_GetServoStatus_Response_right_speed_error(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetServoStatus_Response msg_;
};

class Init_GetServoStatus_Response_right_position_error
{
public:
  explicit Init_GetServoStatus_Response_right_position_error(::basicmicro_ros2::srv::GetServoStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetServoStatus_Response_left_speed_error right_position_error(::basicmicro_ros2::srv::GetServoStatus_Response::_right_position_error_type arg)
  {
    msg_.right_position_error = std::move(arg);
    return Init_GetServoStatus_Response_left_speed_error(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetServoStatus_Response msg_;
};

class Init_GetServoStatus_Response_left_position_error
{
public:
  explicit Init_GetServoStatus_Response_left_position_error(::basicmicro_ros2::srv::GetServoStatus_Response & msg)
  : msg_(msg)
  {}
  Init_GetServoStatus_Response_right_position_error left_position_error(::basicmicro_ros2::srv::GetServoStatus_Response::_left_position_error_type arg)
  {
    msg_.left_position_error = std::move(arg);
    return Init_GetServoStatus_Response_right_position_error(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetServoStatus_Response msg_;
};

class Init_GetServoStatus_Response_success
{
public:
  Init_GetServoStatus_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetServoStatus_Response_left_position_error success(::basicmicro_ros2::srv::GetServoStatus_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetServoStatus_Response_left_position_error(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetServoStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::GetServoStatus_Response>()
{
  return basicmicro_ros2::srv::builder::Init_GetServoStatus_Response_success();
}

}  // namespace basicmicro_ros2


namespace basicmicro_ros2
{

namespace srv
{

namespace builder
{

class Init_GetServoStatus_Event_response
{
public:
  explicit Init_GetServoStatus_Event_response(::basicmicro_ros2::srv::GetServoStatus_Event & msg)
  : msg_(msg)
  {}
  ::basicmicro_ros2::srv::GetServoStatus_Event response(::basicmicro_ros2::srv::GetServoStatus_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetServoStatus_Event msg_;
};

class Init_GetServoStatus_Event_request
{
public:
  explicit Init_GetServoStatus_Event_request(::basicmicro_ros2::srv::GetServoStatus_Event & msg)
  : msg_(msg)
  {}
  Init_GetServoStatus_Event_response request(::basicmicro_ros2::srv::GetServoStatus_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetServoStatus_Event_response(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetServoStatus_Event msg_;
};

class Init_GetServoStatus_Event_info
{
public:
  Init_GetServoStatus_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetServoStatus_Event_request info(::basicmicro_ros2::srv::GetServoStatus_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetServoStatus_Event_request(msg_);
  }

private:
  ::basicmicro_ros2::srv::GetServoStatus_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::basicmicro_ros2::srv::GetServoStatus_Event>()
{
  return basicmicro_ros2::srv::builder::Init_GetServoStatus_Event_info();
}

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__GET_SERVO_STATUS__BUILDER_HPP_
