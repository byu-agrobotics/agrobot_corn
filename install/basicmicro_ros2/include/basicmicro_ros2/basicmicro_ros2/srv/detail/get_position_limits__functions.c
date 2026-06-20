// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from basicmicro_ros2:srv/GetPositionLimits.idl
// generated code does not contain a copyright notice
#include "basicmicro_ros2/srv/detail/get_position_limits__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
basicmicro_ros2__srv__GetPositionLimits_Request__init(basicmicro_ros2__srv__GetPositionLimits_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
basicmicro_ros2__srv__GetPositionLimits_Request__fini(basicmicro_ros2__srv__GetPositionLimits_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
basicmicro_ros2__srv__GetPositionLimits_Request__are_equal(const basicmicro_ros2__srv__GetPositionLimits_Request * lhs, const basicmicro_ros2__srv__GetPositionLimits_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
basicmicro_ros2__srv__GetPositionLimits_Request__copy(
  const basicmicro_ros2__srv__GetPositionLimits_Request * input,
  basicmicro_ros2__srv__GetPositionLimits_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

basicmicro_ros2__srv__GetPositionLimits_Request *
basicmicro_ros2__srv__GetPositionLimits_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__srv__GetPositionLimits_Request * msg = (basicmicro_ros2__srv__GetPositionLimits_Request *)allocator.allocate(sizeof(basicmicro_ros2__srv__GetPositionLimits_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(basicmicro_ros2__srv__GetPositionLimits_Request));
  bool success = basicmicro_ros2__srv__GetPositionLimits_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
basicmicro_ros2__srv__GetPositionLimits_Request__destroy(basicmicro_ros2__srv__GetPositionLimits_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    basicmicro_ros2__srv__GetPositionLimits_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__init(basicmicro_ros2__srv__GetPositionLimits_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__srv__GetPositionLimits_Request * data = NULL;

  if (size) {
    data = (basicmicro_ros2__srv__GetPositionLimits_Request *)allocator.zero_allocate(size, sizeof(basicmicro_ros2__srv__GetPositionLimits_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = basicmicro_ros2__srv__GetPositionLimits_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        basicmicro_ros2__srv__GetPositionLimits_Request__fini(&data[i - 1]);
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
basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__fini(basicmicro_ros2__srv__GetPositionLimits_Request__Sequence * array)
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
      basicmicro_ros2__srv__GetPositionLimits_Request__fini(&array->data[i]);
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

basicmicro_ros2__srv__GetPositionLimits_Request__Sequence *
basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__srv__GetPositionLimits_Request__Sequence * array = (basicmicro_ros2__srv__GetPositionLimits_Request__Sequence *)allocator.allocate(sizeof(basicmicro_ros2__srv__GetPositionLimits_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__destroy(basicmicro_ros2__srv__GetPositionLimits_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__are_equal(const basicmicro_ros2__srv__GetPositionLimits_Request__Sequence * lhs, const basicmicro_ros2__srv__GetPositionLimits_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!basicmicro_ros2__srv__GetPositionLimits_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__copy(
  const basicmicro_ros2__srv__GetPositionLimits_Request__Sequence * input,
  basicmicro_ros2__srv__GetPositionLimits_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(basicmicro_ros2__srv__GetPositionLimits_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    basicmicro_ros2__srv__GetPositionLimits_Request * data =
      (basicmicro_ros2__srv__GetPositionLimits_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!basicmicro_ros2__srv__GetPositionLimits_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          basicmicro_ros2__srv__GetPositionLimits_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!basicmicro_ros2__srv__GetPositionLimits_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `violation_behavior`
#include "rosidl_runtime_c/string_functions.h"

bool
basicmicro_ros2__srv__GetPositionLimits_Response__init(basicmicro_ros2__srv__GetPositionLimits_Response * msg)
{
  if (!msg) {
    return false;
  }
  // limits_enabled
  // left_min_position
  // left_max_position
  // right_min_position
  // right_max_position
  // violation_behavior
  if (!rosidl_runtime_c__String__init(&msg->violation_behavior)) {
    basicmicro_ros2__srv__GetPositionLimits_Response__fini(msg);
    return false;
  }
  // decel_rate
  return true;
}

void
basicmicro_ros2__srv__GetPositionLimits_Response__fini(basicmicro_ros2__srv__GetPositionLimits_Response * msg)
{
  if (!msg) {
    return;
  }
  // limits_enabled
  // left_min_position
  // left_max_position
  // right_min_position
  // right_max_position
  // violation_behavior
  rosidl_runtime_c__String__fini(&msg->violation_behavior);
  // decel_rate
}

bool
basicmicro_ros2__srv__GetPositionLimits_Response__are_equal(const basicmicro_ros2__srv__GetPositionLimits_Response * lhs, const basicmicro_ros2__srv__GetPositionLimits_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // limits_enabled
  if (lhs->limits_enabled != rhs->limits_enabled) {
    return false;
  }
  // left_min_position
  if (lhs->left_min_position != rhs->left_min_position) {
    return false;
  }
  // left_max_position
  if (lhs->left_max_position != rhs->left_max_position) {
    return false;
  }
  // right_min_position
  if (lhs->right_min_position != rhs->right_min_position) {
    return false;
  }
  // right_max_position
  if (lhs->right_max_position != rhs->right_max_position) {
    return false;
  }
  // violation_behavior
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->violation_behavior), &(rhs->violation_behavior)))
  {
    return false;
  }
  // decel_rate
  if (lhs->decel_rate != rhs->decel_rate) {
    return false;
  }
  return true;
}

bool
basicmicro_ros2__srv__GetPositionLimits_Response__copy(
  const basicmicro_ros2__srv__GetPositionLimits_Response * input,
  basicmicro_ros2__srv__GetPositionLimits_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // limits_enabled
  output->limits_enabled = input->limits_enabled;
  // left_min_position
  output->left_min_position = input->left_min_position;
  // left_max_position
  output->left_max_position = input->left_max_position;
  // right_min_position
  output->right_min_position = input->right_min_position;
  // right_max_position
  output->right_max_position = input->right_max_position;
  // violation_behavior
  if (!rosidl_runtime_c__String__copy(
      &(input->violation_behavior), &(output->violation_behavior)))
  {
    return false;
  }
  // decel_rate
  output->decel_rate = input->decel_rate;
  return true;
}

basicmicro_ros2__srv__GetPositionLimits_Response *
basicmicro_ros2__srv__GetPositionLimits_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__srv__GetPositionLimits_Response * msg = (basicmicro_ros2__srv__GetPositionLimits_Response *)allocator.allocate(sizeof(basicmicro_ros2__srv__GetPositionLimits_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(basicmicro_ros2__srv__GetPositionLimits_Response));
  bool success = basicmicro_ros2__srv__GetPositionLimits_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
basicmicro_ros2__srv__GetPositionLimits_Response__destroy(basicmicro_ros2__srv__GetPositionLimits_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    basicmicro_ros2__srv__GetPositionLimits_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__init(basicmicro_ros2__srv__GetPositionLimits_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__srv__GetPositionLimits_Response * data = NULL;

  if (size) {
    data = (basicmicro_ros2__srv__GetPositionLimits_Response *)allocator.zero_allocate(size, sizeof(basicmicro_ros2__srv__GetPositionLimits_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = basicmicro_ros2__srv__GetPositionLimits_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        basicmicro_ros2__srv__GetPositionLimits_Response__fini(&data[i - 1]);
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
basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__fini(basicmicro_ros2__srv__GetPositionLimits_Response__Sequence * array)
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
      basicmicro_ros2__srv__GetPositionLimits_Response__fini(&array->data[i]);
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

basicmicro_ros2__srv__GetPositionLimits_Response__Sequence *
basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__srv__GetPositionLimits_Response__Sequence * array = (basicmicro_ros2__srv__GetPositionLimits_Response__Sequence *)allocator.allocate(sizeof(basicmicro_ros2__srv__GetPositionLimits_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__destroy(basicmicro_ros2__srv__GetPositionLimits_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__are_equal(const basicmicro_ros2__srv__GetPositionLimits_Response__Sequence * lhs, const basicmicro_ros2__srv__GetPositionLimits_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!basicmicro_ros2__srv__GetPositionLimits_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__copy(
  const basicmicro_ros2__srv__GetPositionLimits_Response__Sequence * input,
  basicmicro_ros2__srv__GetPositionLimits_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(basicmicro_ros2__srv__GetPositionLimits_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    basicmicro_ros2__srv__GetPositionLimits_Response * data =
      (basicmicro_ros2__srv__GetPositionLimits_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!basicmicro_ros2__srv__GetPositionLimits_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          basicmicro_ros2__srv__GetPositionLimits_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!basicmicro_ros2__srv__GetPositionLimits_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "basicmicro_ros2/srv/detail/get_position_limits__functions.h"

bool
basicmicro_ros2__srv__GetPositionLimits_Event__init(basicmicro_ros2__srv__GetPositionLimits_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    basicmicro_ros2__srv__GetPositionLimits_Event__fini(msg);
    return false;
  }
  // request
  if (!basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__init(&msg->request, 0)) {
    basicmicro_ros2__srv__GetPositionLimits_Event__fini(msg);
    return false;
  }
  // response
  if (!basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__init(&msg->response, 0)) {
    basicmicro_ros2__srv__GetPositionLimits_Event__fini(msg);
    return false;
  }
  return true;
}

void
basicmicro_ros2__srv__GetPositionLimits_Event__fini(basicmicro_ros2__srv__GetPositionLimits_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__fini(&msg->request);
  // response
  basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__fini(&msg->response);
}

bool
basicmicro_ros2__srv__GetPositionLimits_Event__are_equal(const basicmicro_ros2__srv__GetPositionLimits_Event * lhs, const basicmicro_ros2__srv__GetPositionLimits_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
basicmicro_ros2__srv__GetPositionLimits_Event__copy(
  const basicmicro_ros2__srv__GetPositionLimits_Event * input,
  basicmicro_ros2__srv__GetPositionLimits_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!basicmicro_ros2__srv__GetPositionLimits_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!basicmicro_ros2__srv__GetPositionLimits_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

basicmicro_ros2__srv__GetPositionLimits_Event *
basicmicro_ros2__srv__GetPositionLimits_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__srv__GetPositionLimits_Event * msg = (basicmicro_ros2__srv__GetPositionLimits_Event *)allocator.allocate(sizeof(basicmicro_ros2__srv__GetPositionLimits_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(basicmicro_ros2__srv__GetPositionLimits_Event));
  bool success = basicmicro_ros2__srv__GetPositionLimits_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
basicmicro_ros2__srv__GetPositionLimits_Event__destroy(basicmicro_ros2__srv__GetPositionLimits_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    basicmicro_ros2__srv__GetPositionLimits_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
basicmicro_ros2__srv__GetPositionLimits_Event__Sequence__init(basicmicro_ros2__srv__GetPositionLimits_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__srv__GetPositionLimits_Event * data = NULL;

  if (size) {
    data = (basicmicro_ros2__srv__GetPositionLimits_Event *)allocator.zero_allocate(size, sizeof(basicmicro_ros2__srv__GetPositionLimits_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = basicmicro_ros2__srv__GetPositionLimits_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        basicmicro_ros2__srv__GetPositionLimits_Event__fini(&data[i - 1]);
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
basicmicro_ros2__srv__GetPositionLimits_Event__Sequence__fini(basicmicro_ros2__srv__GetPositionLimits_Event__Sequence * array)
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
      basicmicro_ros2__srv__GetPositionLimits_Event__fini(&array->data[i]);
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

basicmicro_ros2__srv__GetPositionLimits_Event__Sequence *
basicmicro_ros2__srv__GetPositionLimits_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  basicmicro_ros2__srv__GetPositionLimits_Event__Sequence * array = (basicmicro_ros2__srv__GetPositionLimits_Event__Sequence *)allocator.allocate(sizeof(basicmicro_ros2__srv__GetPositionLimits_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = basicmicro_ros2__srv__GetPositionLimits_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
basicmicro_ros2__srv__GetPositionLimits_Event__Sequence__destroy(basicmicro_ros2__srv__GetPositionLimits_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    basicmicro_ros2__srv__GetPositionLimits_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
basicmicro_ros2__srv__GetPositionLimits_Event__Sequence__are_equal(const basicmicro_ros2__srv__GetPositionLimits_Event__Sequence * lhs, const basicmicro_ros2__srv__GetPositionLimits_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!basicmicro_ros2__srv__GetPositionLimits_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
basicmicro_ros2__srv__GetPositionLimits_Event__Sequence__copy(
  const basicmicro_ros2__srv__GetPositionLimits_Event__Sequence * input,
  basicmicro_ros2__srv__GetPositionLimits_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(basicmicro_ros2__srv__GetPositionLimits_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    basicmicro_ros2__srv__GetPositionLimits_Event * data =
      (basicmicro_ros2__srv__GetPositionLimits_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!basicmicro_ros2__srv__GetPositionLimits_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          basicmicro_ros2__srv__GetPositionLimits_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!basicmicro_ros2__srv__GetPositionLimits_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
