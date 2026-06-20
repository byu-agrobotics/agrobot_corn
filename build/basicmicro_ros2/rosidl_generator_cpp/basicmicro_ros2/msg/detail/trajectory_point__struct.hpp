// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from basicmicro_ros2:msg/TrajectoryPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/msg/trajectory_point.hpp"


#ifndef BASICMICRO_ROS2__MSG__DETAIL__TRAJECTORY_POINT__STRUCT_HPP_
#define BASICMICRO_ROS2__MSG__DETAIL__TRAJECTORY_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__basicmicro_ros2__msg__TrajectoryPoint __attribute__((deprecated))
#else
# define DEPRECATED__basicmicro_ros2__msg__TrajectoryPoint __declspec(deprecated)
#endif

namespace basicmicro_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajectoryPoint_
{
  using Type = TrajectoryPoint_<ContainerAllocator>;

  explicit TrajectoryPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command_type = "";
      this->left_distance = 0.0;
      this->right_distance = 0.0;
      this->left_position = 0.0;
      this->right_position = 0.0;
      this->deceleration = 0.0;
      this->speed = 0.0;
      this->acceleration = 0.0;
      this->duration = 0.0;
    }
  }

  explicit TrajectoryPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command_type = "";
      this->left_distance = 0.0;
      this->right_distance = 0.0;
      this->left_position = 0.0;
      this->right_position = 0.0;
      this->deceleration = 0.0;
      this->speed = 0.0;
      this->acceleration = 0.0;
      this->duration = 0.0;
    }
  }

  // field types and members
  using _command_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type_type command_type;
  using _left_distance_type =
    double;
  _left_distance_type left_distance;
  using _right_distance_type =
    double;
  _right_distance_type right_distance;
  using _left_position_type =
    double;
  _left_position_type left_position;
  using _right_position_type =
    double;
  _right_position_type right_position;
  using _deceleration_type =
    double;
  _deceleration_type deceleration;
  using _speed_type =
    double;
  _speed_type speed;
  using _acceleration_type =
    double;
  _acceleration_type acceleration;
  using _duration_type =
    double;
  _duration_type duration;

  // setters for named parameter idiom
  Type & set__command_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command_type = _arg;
    return *this;
  }
  Type & set__left_distance(
    const double & _arg)
  {
    this->left_distance = _arg;
    return *this;
  }
  Type & set__right_distance(
    const double & _arg)
  {
    this->right_distance = _arg;
    return *this;
  }
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
  Type & set__deceleration(
    const double & _arg)
  {
    this->deceleration = _arg;
    return *this;
  }
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__acceleration(
    const double & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__duration(
    const double & _arg)
  {
    this->duration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__basicmicro_ros2__msg__TrajectoryPoint
    std::shared_ptr<basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__basicmicro_ros2__msg__TrajectoryPoint
    std::shared_ptr<basicmicro_ros2::msg::TrajectoryPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajectoryPoint_ & other) const
  {
    if (this->command_type != other.command_type) {
      return false;
    }
    if (this->left_distance != other.left_distance) {
      return false;
    }
    if (this->right_distance != other.right_distance) {
      return false;
    }
    if (this->left_position != other.left_position) {
      return false;
    }
    if (this->right_position != other.right_position) {
      return false;
    }
    if (this->deceleration != other.deceleration) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajectoryPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajectoryPoint_

// alias to use template instance with default allocator
using TrajectoryPoint =
  basicmicro_ros2::msg::TrajectoryPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace basicmicro_ros2

#endif  // BASICMICRO_ROS2__MSG__DETAIL__TRAJECTORY_POINT__STRUCT_HPP_
