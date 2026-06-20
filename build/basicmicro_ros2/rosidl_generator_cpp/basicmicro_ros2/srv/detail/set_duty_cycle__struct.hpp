// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from basicmicro_ros2:srv/SetDutyCycle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/set_duty_cycle.hpp"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__SET_DUTY_CYCLE__STRUCT_HPP_
#define BASICMICRO_ROS2__SRV__DETAIL__SET_DUTY_CYCLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Request __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Request __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetDutyCycle_Request_
{
  using Type = SetDutyCycle_Request_<ContainerAllocator>;

  explicit SetDutyCycle_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_duty = 0;
      this->right_duty = 0;
      this->use_acceleration = false;
      this->acceleration = 0l;
    }
  }

  explicit SetDutyCycle_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_duty = 0;
      this->right_duty = 0;
      this->use_acceleration = false;
      this->acceleration = 0l;
    }
  }

  // field types and members
  using _left_duty_type =
    int16_t;
  _left_duty_type left_duty;
  using _right_duty_type =
    int16_t;
  _right_duty_type right_duty;
  using _use_acceleration_type =
    bool;
  _use_acceleration_type use_acceleration;
  using _acceleration_type =
    int32_t;
  _acceleration_type acceleration;

  // setters for named parameter idiom
  Type & set__left_duty(
    const int16_t & _arg)
  {
    this->left_duty = _arg;
    return *this;
  }
  Type & set__right_duty(
    const int16_t & _arg)
  {
    this->right_duty = _arg;
    return *this;
  }
  Type & set__use_acceleration(
    const bool & _arg)
  {
    this->use_acceleration = _arg;
    return *this;
  }
  Type & set__acceleration(
    const int32_t & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Request
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Request
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetDutyCycle_Request_ & other) const
  {
    if (this->left_duty != other.left_duty) {
      return false;
    }
    if (this->right_duty != other.right_duty) {
      return false;
    }
    if (this->use_acceleration != other.use_acceleration) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetDutyCycle_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetDutyCycle_Request_

// alias to use template instance with default allocator
using SetDutyCycle_Request =
  basicmicro_ros2::srv::SetDutyCycle_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Response __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Response __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetDutyCycle_Response_
{
  using Type = SetDutyCycle_Response_<ContainerAllocator>;

  explicit SetDutyCycle_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetDutyCycle_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Response
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Response
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetDutyCycle_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetDutyCycle_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetDutyCycle_Response_

// alias to use template instance with default allocator
using SetDutyCycle_Response =
  basicmicro_ros2::srv::SetDutyCycle_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Event __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Event __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetDutyCycle_Event_
{
  using Type = SetDutyCycle_Event_<ContainerAllocator>;

  explicit SetDutyCycle_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SetDutyCycle_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::SetDutyCycle_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<basicmicro_ros2::srv::SetDutyCycle_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Event
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__srv__SetDutyCycle_Event
    std::shared_ptr<basicmicro_ros2::srv::SetDutyCycle_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetDutyCycle_Event_ & other) const
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
  bool operator!=(const SetDutyCycle_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetDutyCycle_Event_

// alias to use template instance with default allocator
using SetDutyCycle_Event =
  basicmicro_ros2::srv::SetDutyCycle_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace basicmicro_ros2

namespace basicmicro_ros2
{

namespace srv
{

struct SetDutyCycle
{
  using Request = basicmicro_ros2::srv::SetDutyCycle_Request;
  using Response = basicmicro_ros2::srv::SetDutyCycle_Response;
  using Event = basicmicro_ros2::srv::SetDutyCycle_Event;
};

}  // namespace srv

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__SRV__DETAIL__SET_DUTY_CYCLE__STRUCT_HPP_
