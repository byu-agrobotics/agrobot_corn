// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from basicmicro_ros2:msg/PositionPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/msg/position_point.hpp"


#ifndef BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__STRUCT_HPP_
#define BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__msg__PositionPoint __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__msg__PositionPoint __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PositionPoint_
{
  using Type = PositionPoint_<ContainerAllocator>;

  explicit PositionPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_position = 0.0;
      this->right_position = 0.0;
      this->max_speed = 0.0;
      this->acceleration = 0.0;
      this->deceleration = 0.0;
    }
  }

  explicit PositionPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_position = 0.0;
      this->right_position = 0.0;
      this->max_speed = 0.0;
      this->acceleration = 0.0;
      this->deceleration = 0.0;
    }
  }

  // field types and members
  using _left_position_type =
    double;
  _left_position_type left_position;
  using _right_position_type =
    double;
  _right_position_type right_position;
  using _max_speed_type =
    double;
  _max_speed_type max_speed;
  using _acceleration_type =
    double;
  _acceleration_type acceleration;
  using _deceleration_type =
    double;
  _deceleration_type deceleration;

  // setters for named parameter idiom
  Type & set__left_position(
    const double & _arg)
  {
    this->left_position = _arg;
    return *this;
  }
  Type & set__right_position(
    const double & _arg)
  {
    this->right_position = _arg;
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

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::msg::PositionPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::msg::PositionPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::msg::PositionPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::msg::PositionPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__msg__PositionPoint
    std::shared_ptr<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__msg__PositionPoint
    std::shared_ptr<basicmicro_ros2::msg::PositionPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PositionPoint_ & other) const
  {
    if (this->left_position != other.left_position) {
      return false;
    }
    if (this->right_position != other.right_position) {
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
    return true;
  }
  bool operator!=(const PositionPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PositionPoint_

// alias to use template instance with default allocator
using PositionPoint =
  basicmicro_ros2::msg::PositionPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__MSG__DETAIL__POSITION_POINT__STRUCT_HPP_
