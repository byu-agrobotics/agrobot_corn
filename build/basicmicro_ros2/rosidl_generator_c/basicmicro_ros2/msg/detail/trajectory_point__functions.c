// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from basicmicro_ros2:msg/TrajectoryPoint.idl
// generated code does not contain a copyright notice
#include "basicmicro_ros2/msg/detail/trajectory_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `command_type`
#include "rosidl_runtime_c/string_functions.h"

bool
basicmicro_ros2__msg__TrajectoryPoint__init(basicmicro_ros2__msg__TrajectoryPoint * msg)
{
  if (!msg) {
    return false;
  }
  // command_type
  if (!rosidl_runtime_c__String__init(&msg->command_type)) {
    basicmicro_ros2__msg__TrajectoryPoint__fini(msg);
    return false;
  }
  // left_distance
  // right_distance
  // left_position
  // right_position
  // deceleration
  // speed
  // acceleration
  // duration
  return true;
}

void
basicmicro_ros2__msg__TrajectoryPoint__fini(basicmicro_ros2__msg__TrajectoryPoint * msg)
{
  if (!msg) {
    return;
  }
  // command_type
  rosidl_runtime_c__String__fini(&msg->command_type);
  // left_distance
  // right_distance
  // left_position
  // right_position
  // deceleration
  // speed
  // acceleration
  // duration
}

bool
basicmicro_ros2__msg__TrajectoryPoint__are_equal(const basicmicro_ros2__msg__TrajectoryPoint * lhs, const basicmicro_ros2__msg__TrajectoryPoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command_type), &(rhs->command_type)))
  {
    return false;
  }
  // left_distance
  if (lhs->left_distance != rhs->left_distance) {
    return false;
  }
  // right_distance
  if (lhs->right_distance != rhs->right_distance) {
    return false;
  }
  // left_position
  if (lhs->left_position != rhs->left_position) {
    return false;
  }
  // right_position
  if (lhs->right_position != rhs->right_position) {
    return false;
  }
  // deceleration
  if (lhs->deceleration != rhs->deceleration) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // acceleration
  if (lhs->acceleration != rhs->acceleration) {
    return false;
  }
  // duration
  if (lhs->duration != rhs->duration) {
    return false;
  }
  return true;
}

bool
basicmicro_ros2__msg__TrajectoryPoint__copy(
  const basicmicro_ros2__msg__TrajectoryPoint * input,
  basicmicro_ros2__msg__TrajectoryPoint * output)
{
  if (!input || !output) {
    return false;
  }
  // command_type
  if (!rosidl_runtime_c__String__copy(
      &(input->command_type), &(output->command_type)))
  {
    return false;
  }
  // left_distance
  output->left_distance = input->left_distance;
  // right_distance
  output->right_distance = input->right_distance;
  // left_position
  output->left_position = input->left_position;
  // right_position
  output->right_position = input->right_position;
  // deceleration
  output->deceleration = input->deceleration;
  // speed
  output->speed = input->speed;
  // acceleration
  output->acceleration = input->acceleration;
  // duration
  output->duration = input->duration;
  return true;
}

basicmicro_ros2__msg__TrajectoryPoint *
basicmicro_ros2__msg__TrajectoryPoint__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__msg__TrajectoryPoint * msg = (basicmicro_ros2__msg__TrajectoryPoint *)allocator.allocate(sizeof(basicmicro_ros2__msg__TrajectoryPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(basicmicro_ros2__msg__TrajectoryPoint));
  bool success = basicmicro_ros2__msg__TrajectoryPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
basicmicro_ros2__msg__TrajectoryPoint__destroy(basicmicro_ros2__msg__TrajectoryPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    basicmicro_ros2__msg__TrajectoryPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
basicmicro_ros2__msg__TrajectoryPoint__Sequence__init(basicmicro_ros2__msg__TrajectoryPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__msg__TrajectoryPoint * data = NULL;

  if (size) {
    data = (basicmicro_ros2__msg__TrajectoryPoint *)allocator.zero_allocate(size, sizeof(basicmicro_ros2__msg__TrajectoryPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = basicmicro_ros2__msg__TrajectoryPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        basicmicro_ros2__msg__TrajectoryPoint__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
basicmicro_ros2__msg__TrajectoryPoint__Sequence__fini(basicmicro_ros2__msg__TrajectoryPoint__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      basicmicro_ros2__msg__TrajectoryPoint__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

basicmicro_ros2__msg__TrajectoryPoint__Sequence *
basicmicro_ros2__msg__TrajectoryPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__msg__TrajectoryPoint__Sequence * array = (basicmicro_ros2__msg__TrajectoryPoint__Sequence *)allocator.allocate(sizeof(basicmicro_ros2__msg__TrajectoryPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = basicmicro_ros2__msg__TrajectoryPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
basicmicro_ros2__msg__TrajectoryPoint__Sequence__destroy(basicmicro_ros2__msg__TrajectoryPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    basicmicro_ros2__msg__TrajectoryPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
basicmicro_ros2__msg__TrajectoryPoint__Sequence__are_equal(const basicmicro_ros2__msg__TrajectoryPoint__Sequence * lhs, const basicmicro_ros2__msg__TrajectoryPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!basicmicro_ros2__msg__TrajectoryPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
basicmicro_ros2__msg__TrajectoryPoint__Sequence__copy(
  const basicmicro_ros2__msg__TrajectoryPoint__Sequence * input,
  basicmicro_ros2__msg__TrajectoryPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(basicmicro_ros2__msg__TrajectoryPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    basicmicro_ros2__msg__TrajectoryPoint * data =
      (basicmicro_ros2__msg__TrajectoryPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!basicmicro_ros2__msg__TrajectoryPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          basicmicro_ros2__msg__TrajectoryPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!basicmicro_ros2__msg__TrajectoryPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
