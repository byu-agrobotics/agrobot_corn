// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from basicmicro_ros2:srv/ExecutePositionSequence.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/execute_position_sequence.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_POSITION_SEQUENCE__STRUCT_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_POSITION_SEQUENCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position_points'
#include "basicmicro_ros2/msg/detail/position_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Request __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Request __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ExecutePositionSequence_Request_
{
  using Type = ExecutePositionSequence_Request_<ContainerAllocator>;

  explicit ExecutePositionSequence_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ExecutePositionSequence_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _position_points_type =
    std::vector<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator>>>;
  _position_points_type position_points;

  // setters for named parameter idiom
  Type & set__position_points(
    const std::vector<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator>>> & _arg)
  {
    this->position_points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Request
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Request
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecutePositionSequence_Request_ & other) const
  {
    if (this->position_points != other.position_points) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecutePositionSequence_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecutePositionSequence_Request_

// alias to use template instance with default allocator
using ExecutePositionSequence_Request =
  basicmicro_ros2::srv::ExecutePositionSequence_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Response __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Response __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ExecutePositionSequence_Response_
{
  using Type = ExecutePositionSequence_Response_<ContainerAllocator>;

  explicit ExecutePositionSequence_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->total_commands_sent = 0l;
    }
  }

  explicit ExecutePositionSequence_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->total_commands_sent = 0l;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _total_commands_sent_type =
    int32_t;
  _total_commands_sent_type total_commands_sent;

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
  Type & set__total_commands_sent(
    const int32_t & _arg)
  {
    this->total_commands_sent = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Response
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Response
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecutePositionSequence_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->total_commands_sent != other.total_commands_sent) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecutePositionSequence_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecutePositionSequence_Response_

// alias to use template instance with default allocator
using ExecutePositionSequence_Response =
  basicmicro_ros2::srv::ExecutePositionSequence_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Event __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Event __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ExecutePositionSequence_Event_
{
  using Type = ExecutePositionSequence_Event_<ContainerAllocator>;

  explicit ExecutePositionSequence_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ExecutePositionSequence_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::ExecutePositionSequence_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::ExecutePositionSequence_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Event
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__ExecutePositionSequence_Event
    std::shared_ptr<basicmicro_ros2::srv::ExecutePositionSequence_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecutePositionSequence_Event_ & other) const
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
  bool operator!=(const ExecutePositionSequence_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecutePositionSequence_Event_

// alias to use template instance with default allocator
using ExecutePositionSequence_Event =
  basicmicro_ros2::srv::ExecutePositionSequence_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2

namespace basicmicro_ros2
{

namespace srv
{

struct ExecutePositionSequence
{
  using Request = basicmicro_ros2::srv::ExecutePositionSequence_Request;
  using Response = basicmicro_ros2::srv::ExecutePositionSequence_Response;
  using Event = basicmicro_ros2::srv::ExecutePositionSequence_Event;
};

}  // namespace srv

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_POSITION_SEQUENCE__STRUCT_HPP_
