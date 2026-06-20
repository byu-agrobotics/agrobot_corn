// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from basicmicro_ros2:srv/MoveToAbsolutePosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/move_to_absolute_position.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__MOVE_TO_ABSOLUTE_POSITION__STRUCT_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__MOVE_TO_ABSOLUTE_POSITION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Request __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Request __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveToAbsolutePosition_Request_
{
  using Type = MoveToAbsolutePosition_Request_<ContainerAllocator>;

  explicit MoveToAbsolutePosition_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_position_radians = 0.0;
      this->right_position_radians = 0.0;
      this->max_speed = 0.0;
      this->acceleration = 0.0;
      this->deceleration = 0.0;
      this->buffer_command = false;
    }
  }

  explicit MoveToAbsolutePosition_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_position_radians = 0.0;
      this->right_position_radians = 0.0;
      this->max_speed = 0.0;
      this->acceleration = 0.0;
      this->deceleration = 0.0;
      this->buffer_command = false;
    }
  }

  // field types and members
  using _left_position_radians_type =
    double;
  _left_position_radians_type left_position_radians;
  using _right_position_radians_type =
    double;
  _right_position_radians_type right_position_radians;
  using _max_speed_type =
    double;
  _max_speed_type max_speed;
  using _acceleration_type =
    double;
  _acceleration_type acceleration;
  using _deceleration_type =
    double;
  _deceleration_type deceleration;
  using _buffer_command_type =
    bool;
  _buffer_command_type buffer_command;

  // setters for named parameter idiom
  Type & set__left_position_radians(
    const double & _arg)
  {
    this->left_position_radians = _arg;
    return *this;
  }
  Type & set__right_position_radians(
    const double & _arg)
  {
    this->right_position_radians = _arg;
    return *this;
  }
  Type & set__max_speed(
    const double & _arg)
  {
    this->max_speed = _arg;
    return *this;
  }
  Type & set__acceleration(
    const double & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__deceleration(
    const double & _arg)
  {
    this->deceleration = _arg;
    return *this;
  }
  Type & set__buffer_command(
    const bool & _arg)
  {
    this->buffer_command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Request
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Request
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToAbsolutePosition_Request_ & other) const
  {
    if (this->left_position_radians != other.left_position_radians) {
      return false;
    }
    if (this->right_position_radians != other.right_position_radians) {
      return false;
    }
    if (this->max_speed != other.max_speed) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->deceleration != other.deceleration) {
      return false;
    }
    if (this->buffer_command != other.buffer_command) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToAbsolutePosition_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToAbsolutePosition_Request_

// alias to use template instance with default allocator
using MoveToAbsolutePosition_Request =
  basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Response __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Response __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveToAbsolutePosition_Response_
{
  using Type = MoveToAbsolutePosition_Response_<ContainerAllocator>;

  explicit MoveToAbsolutePosition_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit MoveToAbsolutePosition_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Response
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Response
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToAbsolutePosition_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToAbsolutePosition_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToAbsolutePosition_Response_

// alias to use template instance with default allocator
using MoveToAbsolutePosition_Response =
  basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Event __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Event __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveToAbsolutePosition_Event_
{
  using Type = MoveToAbsolutePosition_Event_<ContainerAllocator>;

  explicit MoveToAbsolutePosition_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit MoveToAbsolutePosition_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::MoveToAbsolutePosition_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::MoveToAbsolutePosition_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Event
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__MoveToAbsolutePosition_Event
    std::shared_ptr<basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToAbsolutePosition_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToAbsolutePosition_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToAbsolutePosition_Event_

// alias to use template instance with default allocator
using MoveToAbsolutePosition_Event =
  basicmicro_ros2::srv::MoveToAbsolutePosition_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2

namespace basicmicro_ros2
{

namespace srv
{

struct MoveToAbsolutePosition
{
  using Request = basicmicro_ros2::srv::MoveToAbsolutePosition_Request;
  using Response = basicmicro_ros2::srv::MoveToAbsolutePosition_Response;
  using Event = basicmicro_ros2::srv::MoveToAbsolutePosition_Event;
};

}  // namespace srv

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__MOVE_TO_ABSOLUTE_POSITION__STRUCT_HPP_
