// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from basicmicro_ros2:srv/ExecuteTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "basicmicro_ros2/srv/execute_trajectory.h"


#ifndef BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_TRAJECTORY__STRUCT_H_
#define BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'trajectory_points'
#include "basicmicro_ros2/msg/detail/trajectory_point__struct.h"
// Member 'trajectory_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ExecuteTrajectory in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__ExecuteTrajectory_Request
{
  /// Trajectory points to execute in sequence
  basicmicro_ros2__msg__TrajectoryPoint__Sequence trajectory_points;
  /// Trajectory type - controls validation and execution strategy
  /// 'distance', 'position', or 'mixed'
  rosidl_runtime_c__String trajectory_type;
} basicmicro_ros2__srv__ExecuteTrajectory_Request;

// Struct for a sequence of basicmicro_ros2__srv__ExecuteTrajectory_Request.
typedef struct basicmicro_ros2__srv__ExecuteTrajectory_Request__Sequence
{
  basicmicro_ros2__srv__ExecuteTrajectory_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__ExecuteTrajectory_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ExecuteTrajectory in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__ExecuteTrajectory_Response
{
  /// True if trajectory was successfully queued
  bool success;
  /// Status message or error description
  rosidl_runtime_c__String message;
  /// Number of commands successfully sent to buffer
  int32_t total_commands_sent;
} basicmicro_ros2__srv__ExecuteTrajectory_Response;

// Struct for a sequence of basicmicro_ros2__srv__ExecuteTrajectory_Response.
typedef struct basicmicro_ros2__srv__ExecuteTrajectory_Response__Sequence
{
  basicmicro_ros2__srv__ExecuteTrajectory_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__ExecuteTrajectory_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  basicmicro_ros2__srv__ExecuteTrajectory_Event__request__MAX_SIZE = 1
};
// response
enum
{
  basicmicro_ros2__srv__ExecuteTrajectory_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ExecuteTrajectory in the package basicmicro_ros2.
typedef struct basicmicro_ros2__srv__ExecuteTrajectory_Event
{
  service_msgs__msg__ServiceEventInfo info;
  basicmicro_ros2__srv__ExecuteTrajectory_Request__Sequence request;
  basicmicro_ros2__srv__ExecuteTrajectory_Response__Sequence response;
} basicmicro_ros2__srv__ExecuteTrajectory_Event;

// Struct for a sequence of basicmicro_ros2__srv__ExecuteTrajectory_Event.
typedef struct basicmicro_ros2__srv__ExecuteTrajectory_Event__Sequence
{
  basicmicro_ros2__srv__ExecuteTrajectory_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} basicmicro_ros2__srv__ExecuteTrajectory_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASICMICRO_ROS2__SRV__DETAIL__EXECUTE_TRAJECTORY__STRUCT_H_
