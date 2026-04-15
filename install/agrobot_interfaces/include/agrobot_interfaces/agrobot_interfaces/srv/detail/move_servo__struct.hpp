// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from agrobot_interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "agrobot_interfaces/srv/move_servo.hpp"


#ifndef AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_HPP_
#define AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__agrobot_interfaces__srv__MoveServo_Request __attribute__((deprecated))
#else
# define DEPRECATED__agrobot_interfaces__srv__MoveServo_Request __declspec(deprecated)
#endif

namespace agrobot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveServo_Request_
{
  using Type = MoveServo_Request_<ContainerAllocator>;

  explicit MoveServo_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = "";
    }
  }

  explicit MoveServo_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = "";
    }
  }

  // field types and members
  using _request_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__agrobot_interfaces__srv__MoveServo_Request
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__agrobot_interfaces__srv__MoveServo_Request
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveServo_Request_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveServo_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveServo_Request_

// alias to use template instance with default allocator
using MoveServo_Request =
  agrobot_interfaces::srv::MoveServo_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace agrobot_interfaces


#ifndef _WIN32
# define DEPRECATED__agrobot_interfaces__srv__MoveServo_Response __attribute__((deprecated))
#else
# define DEPRECATED__agrobot_interfaces__srv__MoveServo_Response __declspec(deprecated)
#endif

namespace agrobot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveServo_Response_
{
  using Type = MoveServo_Response_<ContainerAllocator>;

  explicit MoveServo_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = "";
    }
  }

  explicit MoveServo_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = "";
    }
  }

  // field types and members
  using _response_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__response(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__agrobot_interfaces__srv__MoveServo_Response
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__agrobot_interfaces__srv__MoveServo_Response
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveServo_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveServo_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveServo_Response_

// alias to use template instance with default allocator
using MoveServo_Response =
  agrobot_interfaces::srv::MoveServo_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace agrobot_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__agrobot_interfaces__srv__MoveServo_Event __attribute__((deprecated))
#else
# define DEPRECATED__agrobot_interfaces__srv__MoveServo_Event __declspec(deprecated)
#endif

namespace agrobot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveServo_Event_
{
  using Type = MoveServo_Event_<ContainerAllocator>;

  explicit MoveServo_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit MoveServo_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<agrobot_interfaces::srv::MoveServo_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<agrobot_interfaces::srv::MoveServo_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__agrobot_interfaces__srv__MoveServo_Event
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__agrobot_interfaces__srv__MoveServo_Event
    std::shared_ptr<agrobot_interfaces::srv::MoveServo_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveServo_Event_ & other) const
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
  bool operator!=(const MoveServo_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveServo_Event_

// alias to use template instance with default allocator
using MoveServo_Event =
  agrobot_interfaces::srv::MoveServo_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace agrobot_interfaces

namespace agrobot_interfaces
{

namespace srv
{

struct MoveServo
{
  using Request = agrobot_interfaces::srv::MoveServo_Request;
  using Response = agrobot_interfaces::srv::MoveServo_Response;
  using Event = agrobot_interfaces::srv::MoveServo_Event;
};

}  // namespace srv

}  // namespace agrobot_interfaces

#endif  // AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_HPP_
