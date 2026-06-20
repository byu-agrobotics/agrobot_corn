// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from basicmicro_ros2:msg/PositionPoint.idl
// generated code does not contain a copyright notice
#include "basicmicro_ros2/msg/detail/position_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
basicmicro_ros2__msg__PositionPoint__init(basicmicro_ros2__msg__PositionPoint * msg)
{
  if (!msg) {
    return false;
  }
  // left_position
  // right_position
  // max_speed
  // acceleration
  // deceleration
  return true;
}

void
basicmicro_ros2__msg__PositionPoint__fini(basicmicro_ros2__msg__PositionPoint * msg)
{
  if (!msg) {
    return;
  }
  // left_position
  // right_position
  // max_speed
  // acceleration
  // deceleration
}

bool
basicmicro_ros2__msg__PositionPoint__are_equal(const basicmicro_ros2__msg__PositionPoint * lhs, const basicmicro_ros2__msg__PositionPoint * rhs)
{
  if (!lhs || !rhs) {
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
  // max_speed
  if (lhs->max_speed != rhs->max_speed) {
    return false;
  }
  // acceleration
  if (lhs->acceleration != rhs->acceleration) {
    return false;
  }
  // deceleration
  if (lhs->deceleration != rhs->deceleration) {
    return false;
  }
  return true;
}

bool
basicmicro_ros2__msg__PositionPoint__copy(
  const basicmicro_ros2__msg__PositionPoint * input,
  basicmicro_ros2__msg__PositionPoint * output)
{
  if (!input || !output) {
    return false;
  }
  // left_position
  output->left_position = input->left_position;
  // right_position
  output->right_position = input->right_position;
  // max_speed
  output->max_speed = input->max_speed;
  // acceleration
  output->acceleration = input->acceleration;
  // deceleration
  output->deceleration = input->deceleration;
  return true;
}

basicmicro_ros2__msg__PositionPoint *
basicmicro_ros2__msg__PositionPoint__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__msg__PositionPoint * msg = (basicmicro_ros2__msg__PositionPoint *)allocator.allocate(sizeof(basicmicro_ros2__msg__PositionPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(basicmicro_ros2__msg__PositionPoint));
  bool success = basicmicro_ros2__msg__PositionPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
basicmicro_ros2__msg__PositionPoint__destroy(basicmicro_ros2__msg__PositionPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    basicmicro_ros2__msg__PositionPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
basicmicro_ros2__msg__PositionPoint__Sequence__init(basicmicro_ros2__msg__PositionPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__msg__PositionPoint * data = NULL;

  if (size) {
    data = (basicmicro_ros2__msg__PositionPoint *)allocator.zero_allocate(size, sizeof(basicmicro_ros2__msg__PositionPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = basicmicro_ros2__msg__PositionPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        basicmicro_ros2__msg__PositionPoint__fini(&data[i - 1]);
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
basicmicro_ros2__msg__PositionPoint__Sequence__fini(basicmicro_ros2__msg__PositionPoint__Sequence * array)
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
      basicmicro_ros2__msg__PositionPoint__fini(&array->data[i]);
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

basicmicro_ros2__msg__PositionPoint__Sequence *
basicmicro_ros2__msg__PositionPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__msg__PositionPoint__Sequence * array = (basicmicro_ros2__msg__PositionPoint__Sequence *)allocator.allocate(sizeof(basicmicro_ros2__msg__PositionPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = basicmicro_ros2__msg__PositionPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
basicmicro_ros2__msg__PositionPoint__Sequence__destroy(basicmicro_ros2__msg__PositionPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    basicmicro_ros2__msg__PositionPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
basicmicro_ros2__msg__PositionPoint__Sequence__are_equal(const basicmicro_ros2__msg__PositionPoint__Sequence * lhs, const basicmicro_ros2__msg__PositionPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!basicmicro_ros2__msg__PositionPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
basicmicro_ros2__msg__PositionPoint__Sequence__copy(
  const basicmicro_ros2__msg__PositionPoint__Sequence * input,
  basicmicro_ros2__msg__PositionPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(basicmicro_ros2__msg__PositionPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    basicmicro_ros2__msg__PositionPoint * data =
      (basicmicro_ros2__msg__PositionPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!basicmicro_ros2__msg__PositionPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          basicmicro_ros2__msg__PositionPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!basicmicro_ros2__msg__PositionPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
