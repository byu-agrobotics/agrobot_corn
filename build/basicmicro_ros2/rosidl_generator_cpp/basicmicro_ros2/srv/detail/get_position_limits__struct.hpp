// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from basicmicro_ros2:srv/GetPositionLimits.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/get_position_limits.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__GET_POSITION_LIMITS__STRUCT_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__GET_POSITION_LIMITS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Request __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Request __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPositionLimits_Request_
{
  using Type = GetPositionLimits_Request_<ContainerAllocator>;

  explicit GetPositionLimits_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetPositionLimits_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Request
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Request
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPositionLimits_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPositionLimits_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPositionLimits_Request_

// alias to use template instance with default allocator
using GetPositionLimits_Request =
  basicmicro_ros2::srv::GetPositionLimits_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Response __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Response __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPositionLimits_Response_
{
  using Type = GetPositionLimits_Response_<ContainerAllocator>;

  explicit GetPositionLimits_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->limits_enabled = false;
      this->left_min_position = 0.0;
      this->left_max_position = 0.0;
      this->right_min_position = 0.0;
      this->right_max_position = 0.0;
      this->violation_behavior = "";
      this->decel_rate = 0.0;
    }
  }

  explicit GetPositionLimits_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : violation_behavior(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->limits_enabled = false;
      this->left_min_position = 0.0;
      this->left_max_position = 0.0;
      this->right_min_position = 0.0;
      this->right_max_position = 0.0;
      this->violation_behavior = "";
      this->decel_rate = 0.0;
    }
  }

  // field types and members
  using _limits_enabled_type =
    bool;
  _limits_enabled_type limits_enabled;
  using _left_min_position_type =
    double;
  _left_min_position_type left_min_position;
  using _left_max_position_type =
    double;
  _left_max_position_type left_max_position;
  using _right_min_position_type =
    double;
  _right_min_position_type right_min_position;
  using _right_max_position_type =
    double;
  _right_max_position_type right_max_position;
  using _violation_behavior_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _violation_behavior_type violation_behavior;
  using _decel_rate_type =
    double;
  _decel_rate_type decel_rate;

  // setters for named parameter idiom
  Type & set__limits_enabled(
    const bool & _arg)
  {
    this->limits_enabled = _arg;
    return *this;
  }
  Type & set__left_min_position(
    const double & _arg)
  {
    this->left_min_position = _arg;
    return *this;
  }
  Type & set__left_max_position(
    const double & _arg)
  {
    this->left_max_position = _arg;
    return *this;
  }
  Type & set__right_min_position(
    const double & _arg)
  {
    this->right_min_position = _arg;
    return *this;
  }
  Type & set__right_max_position(
    const double & _arg)
  {
    this->right_max_position = _arg;
    return *this;
  }
  Type & set__violation_behavior(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->violation_behavior = _arg;
    return *this;
  }
  Type & set__decel_rate(
    const double & _arg)
  {
    this->decel_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Response
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Response
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPositionLimits_Response_ & other) const
  {
    if (this->limits_enabled != other.limits_enabled) {
      return false;
    }
    if (this->left_min_position != other.left_min_position) {
      return false;
    }
    if (this->left_max_position != other.left_max_position) {
      return false;
    }
    if (this->right_min_position != other.right_min_position) {
      return false;
    }
    if (this->right_max_position != other.right_max_position) {
      return false;
    }
    if (this->violation_behavior != other.violation_behavior) {
      return false;
    }
    if (this->decel_rate != other.decel_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPositionLimits_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPositionLimits_Response_

// alias to use template instance with default allocator
using GetPositionLimits_Response =
  basicmicro_ros2::srv::GetPositionLimits_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Event __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Event __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPositionLimits_Event_
{
  using Type = GetPositionLimits_Event_<ContainerAllocator>;

  explicit GetPositionLimits_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetPositionLimits_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::GetPositionLimits_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::GetPositionLimits_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Event
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__GetPositionLimits_Event
    std::shared_ptr<basicmicro_ros2::srv::GetPositionLimits_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPositionLimits_Event_ & other) const
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
  bool operator!=(const GetPositionLimits_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPositionLimits_Event_

// alias to use template instance with default allocator
using GetPositionLimits_Event =
  basicmicro_ros2::srv::GetPositionLimits_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2

namespace basicmicro_ros2
{

namespace srv
{

struct GetPositionLimits
{
  using Request = basicmicro_ros2::srv::GetPositionLimits_Request;
  using Response = basicmicro_ros2::srv::GetPositionLimits_Response;
  using Event = basicmicro_ros2::srv::GetPositionLimits_Event;
};

}  // namespace srv

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__GET_POSITION_LIMITS__STRUCT_HPP_
