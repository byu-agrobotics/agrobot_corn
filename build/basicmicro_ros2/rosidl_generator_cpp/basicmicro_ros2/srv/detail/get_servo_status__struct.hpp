// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from basicmicro_ros2:srv/GetServoStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/get_servo_status.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__GET_SERVO_STATUS__STRUCT_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__GET_SERVO_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Request __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Request __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetServoStatus_Request_
{
  using Type = GetServoStatus_Request_<ContainerAllocator>;

  explicit GetServoStatus_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetServoStatus_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Request
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Request
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetServoStatus_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetServoStatus_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetServoStatus_Request_

// alias to use template instance with default allocator
using GetServoStatus_Request =
  basicmicro_ros2::srv::GetServoStatus_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Response __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Response __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetServoStatus_Response_
{
  using Type = GetServoStatus_Response_<ContainerAllocator>;

  explicit GetServoStatus_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->left_position_error = 0l;
      this->right_position_error = 0l;
      this->left_speed_error = 0l;
      this->right_speed_error = 0l;
      this->error_limits_exceeded = false;
      this->message = "";
    }
  }

  explicit GetServoStatus_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->left_position_error = 0l;
      this->right_position_error = 0l;
      this->left_speed_error = 0l;
      this->right_speed_error = 0l;
      this->error_limits_exceeded = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _left_position_error_type =
    int32_t;
  _left_position_error_type left_position_error;
  using _right_position_error_type =
    int32_t;
  _right_position_error_type right_position_error;
  using _left_speed_error_type =
    int32_t;
  _left_speed_error_type left_speed_error;
  using _right_speed_error_type =
    int32_t;
  _right_speed_error_type right_speed_error;
  using _error_limits_exceeded_type =
    bool;
  _error_limits_exceeded_type error_limits_exceeded;
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
  Type & set__left_position_error(
    const int32_t & _arg)
  {
    this->left_position_error = _arg;
    return *this;
  }
  Type & set__right_position_error(
    const int32_t & _arg)
  {
    this->right_position_error = _arg;
    return *this;
  }
  Type & set__left_speed_error(
    const int32_t & _arg)
  {
    this->left_speed_error = _arg;
    return *this;
  }
  Type & set__right_speed_error(
    const int32_t & _arg)
  {
    this->right_speed_error = _arg;
    return *this;
  }
  Type & set__error_limits_exceeded(
    const bool & _arg)
  {
    this->error_limits_exceeded = _arg;
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
    basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Response
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Response
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetServoStatus_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->left_position_error != other.left_position_error) {
      return false;
    }
    if (this->right_position_error != other.right_position_error) {
      return false;
    }
    if (this->left_speed_error != other.left_speed_error) {
      return false;
    }
    if (this->right_speed_error != other.right_speed_error) {
      return false;
    }
    if (this->error_limits_exceeded != other.error_limits_exceeded) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetServoStatus_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetServoStatus_Response_

// alias to use template instance with default allocator
using GetServoStatus_Response =
  basicmicro_ros2::srv::GetServoStatus_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Event __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Event __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetServoStatus_Event_
{
  using Type = GetServoStatus_Event_<ContainerAllocator>;

  explicit GetServoStatus_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetServoStatus_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::GetServoStatus_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::GetServoStatus_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Event
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__GetServoStatus_Event
    std::shared_ptr<basicmicro_ros2::srv::GetServoStatus_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetServoStatus_Event_ & other) const
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
  bool operator!=(const GetServoStatus_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetServoStatus_Event_

// alias to use template instance with default allocator
using GetServoStatus_Event =
  basicmicro_ros2::srv::GetServoStatus_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2

namespace basicmicro_ros2
{

namespace srv
{

struct GetServoStatus
{
  using Request = basicmicro_ros2::srv::GetServoStatus_Request;
  using Response = basicmicro_ros2::srv::GetServoStatus_Response;
  using Event = basicmicro_ros2::srv::GetServoStatus_Event;
};

}  // namespace srv

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__GET_SERVO_STATUS__STRUCT_HPP_
