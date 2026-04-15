// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from agrobot_interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice
#include "agrobot_interfaces/srv/detail/move_servo__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `request`
#include "rosidl_runtime_c/string_functions.h"

bool
agrobot_interfaces__srv__MoveServo_Request__init(agrobot_interfaces__srv__MoveServo_Request * msg)
{
  if (!msg) {
    return false;
  }
  // request
  if (!rosidl_runtime_c__String__init(&msg->request)) {
    agrobot_interfaces__srv__MoveServo_Request__fini(msg);
    return false;
  }
  return true;
}

void
agrobot_interfaces__srv__MoveServo_Request__fini(agrobot_interfaces__srv__MoveServo_Request * msg)
{
  if (!msg) {
    return;
  }
  // request
  rosidl_runtime_c__String__fini(&msg->request);
}

bool
agrobot_interfaces__srv__MoveServo_Request__are_equal(const agrobot_interfaces__srv__MoveServo_Request * lhs, const agrobot_interfaces__srv__MoveServo_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // request
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  return true;
}

bool
agrobot_interfaces__srv__MoveServo_Request__copy(
  const agrobot_interfaces__srv__MoveServo_Request * input,
  agrobot_interfaces__srv__MoveServo_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // request
  if (!rosidl_runtime_c__String__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  return true;
}

agrobot_interfaces__srv__MoveServo_Request *
agrobot_interfaces__srv__MoveServo_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrobot_interfaces__srv__MoveServo_Request * msg = (agrobot_interfaces__srv__MoveServo_Request *)allocator.allocate(sizeof(agrobot_interfaces__srv__MoveServo_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(agrobot_interfaces__srv__MoveServo_Request));
  bool success = agrobot_interfaces__srv__MoveServo_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
agrobot_interfaces__srv__MoveServo_Request__destroy(agrobot_interfaces__srv__MoveServo_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    agrobot_interfaces__srv__MoveServo_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
agrobot_interfaces__srv__MoveServo_Request__Sequence__init(agrobot_interfaces__srv__MoveServo_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrobot_interfaces__srv__MoveServo_Request * data = NULL;

  if (size) {
    data = (agrobot_interfaces__srv__MoveServo_Request *)allocator.zero_allocate(size, sizeof(agrobot_interfaces__srv__MoveServo_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = agrobot_interfaces__srv__MoveServo_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        agrobot_interfaces__srv__MoveServo_Request__fini(&data[i - 1]);
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
agrobot_interfaces__srv__MoveServo_Request__Sequence__fini(agrobot_interfaces__srv__MoveServo_Request__Sequence * array)
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
      agrobot_interfaces__srv__MoveServo_Request__fini(&array->data[i]);
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

agrobot_interfaces__srv__MoveServo_Request__Sequence *
agrobot_interfaces__srv__MoveServo_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrobot_interfaces__srv__MoveServo_Request__Sequence * array = (agrobot_interfaces__srv__MoveServo_Request__Sequence *)allocator.allocate(sizeof(agrobot_interfaces__srv__MoveServo_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = agrobot_interfaces__srv__MoveServo_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
agrobot_interfaces__srv__MoveServo_Request__Sequence__destroy(agrobot_interfaces__srv__MoveServo_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    agrobot_interfaces__srv__MoveServo_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
agrobot_interfaces__srv__MoveServo_Request__Sequence__are_equal(const agrobot_interfaces__srv__MoveServo_Request__Sequence * lhs, const agrobot_interfaces__srv__MoveServo_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!agrobot_interfaces__srv__MoveServo_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
agrobot_interfaces__srv__MoveServo_Request__Sequence__copy(
  const agrobot_interfaces__srv__MoveServo_Request__Sequence * input,
  agrobot_interfaces__srv__MoveServo_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(agrobot_interfaces__srv__MoveServo_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    agrobot_interfaces__srv__MoveServo_Request * data =
      (agrobot_interfaces__srv__MoveServo_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!agrobot_interfaces__srv__MoveServo_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          agrobot_interfaces__srv__MoveServo_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!agrobot_interfaces__srv__MoveServo_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `response`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
agrobot_interfaces__srv__MoveServo_Response__init(agrobot_interfaces__srv__MoveServo_Response * msg)
{
  if (!msg) {
    return false;
  }
  // response
  if (!rosidl_runtime_c__String__init(&msg->response)) {
    agrobot_interfaces__srv__MoveServo_Response__fini(msg);
    return false;
  }
  return true;
}

void
agrobot_interfaces__srv__MoveServo_Response__fini(agrobot_interfaces__srv__MoveServo_Response * msg)
{
  if (!msg) {
    return;
  }
  // response
  rosidl_runtime_c__String__fini(&msg->response);
}

bool
agrobot_interfaces__srv__MoveServo_Response__are_equal(const agrobot_interfaces__srv__MoveServo_Response * lhs, const agrobot_interfaces__srv__MoveServo_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // response
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
agrobot_interfaces__srv__MoveServo_Response__copy(
  const agrobot_interfaces__srv__MoveServo_Response * input,
  agrobot_interfaces__srv__MoveServo_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // response
  if (!rosidl_runtime_c__String__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

agrobot_interfaces__srv__MoveServo_Response *
agrobot_interfaces__srv__MoveServo_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrobot_interfaces__srv__MoveServo_Response * msg = (agrobot_interfaces__srv__MoveServo_Response *)allocator.allocate(sizeof(agrobot_interfaces__srv__MoveServo_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(agrobot_interfaces__srv__MoveServo_Response));
  bool success = agrobot_interfaces__srv__MoveServo_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
agrobot_interfaces__srv__MoveServo_Response__destroy(agrobot_interfaces__srv__MoveServo_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    agrobot_interfaces__srv__MoveServo_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
agrobot_interfaces__srv__MoveServo_Response__Sequence__init(agrobot_interfaces__srv__MoveServo_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrobot_interfaces__srv__MoveServo_Response * data = NULL;

  if (size) {
    data = (agrobot_interfaces__srv__MoveServo_Response *)allocator.zero_allocate(size, sizeof(agrobot_interfaces__srv__MoveServo_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = agrobot_interfaces__srv__MoveServo_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        agrobot_interfaces__srv__MoveServo_Response__fini(&data[i - 1]);
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
agrobot_interfaces__srv__MoveServo_Response__Sequence__fini(agrobot_interfaces__srv__MoveServo_Response__Sequence * array)
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
      agrobot_interfaces__srv__MoveServo_Response__fini(&array->data[i]);
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

agrobot_interfaces__srv__MoveServo_Response__Sequence *
agrobot_interfaces__srv__MoveServo_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrobot_interfaces__srv__MoveServo_Response__Sequence * array = (agrobot_interfaces__srv__MoveServo_Response__Sequence *)allocator.allocate(sizeof(agrobot_interfaces__srv__MoveServo_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = agrobot_interfaces__srv__MoveServo_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
agrobot_interfaces__srv__MoveServo_Response__Sequence__destroy(agrobot_interfaces__srv__MoveServo_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    agrobot_interfaces__srv__MoveServo_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
agrobot_interfaces__srv__MoveServo_Response__Sequence__are_equal(const agrobot_interfaces__srv__MoveServo_Response__Sequence * lhs, const agrobot_interfaces__srv__MoveServo_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!agrobot_interfaces__srv__MoveServo_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
agrobot_interfaces__srv__MoveServo_Response__Sequence__copy(
  const agrobot_interfaces__srv__MoveServo_Response__Sequence * input,
  agrobot_interfaces__srv__MoveServo_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(agrobot_interfaces__srv__MoveServo_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    agrobot_interfaces__srv__MoveServo_Response * data =
      (agrobot_interfaces__srv__MoveServo_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!agrobot_interfaces__srv__MoveServo_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          agrobot_interfaces__srv__MoveServo_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!agrobot_interfaces__srv__MoveServo_Response__copy(
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
// #include "agrobot_interfaces/srv/detail/move_servo__functions.h"

bool
agrobot_interfaces__srv__MoveServo_Event__init(agrobot_interfaces__srv__MoveServo_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    agrobot_interfaces__srv__MoveServo_Event__fini(msg);
    return false;
  }
  // request
  if (!agrobot_interfaces__srv__MoveServo_Request__Sequence__init(&msg->request, 0)) {
    agrobot_interfaces__srv__MoveServo_Event__fini(msg);
    return false;
  }
  // response
  if (!agrobot_interfaces__srv__MoveServo_Response__Sequence__init(&msg->response, 0)) {
    agrobot_interfaces__srv__MoveServo_Event__fini(msg);
    return false;
  }
  return true;
}

void
agrobot_interfaces__srv__MoveServo_Event__fini(agrobot_interfaces__srv__MoveServo_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  agrobot_interfaces__srv__MoveServo_Request__Sequence__fini(&msg->request);
  // response
  agrobot_interfaces__srv__MoveServo_Response__Sequence__fini(&msg->response);
}

bool
agrobot_interfaces__srv__MoveServo_Event__are_equal(const agrobot_interfaces__srv__MoveServo_Event * lhs, const agrobot_interfaces__srv__MoveServo_Event * rhs)
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
  if (!agrobot_interfaces__srv__MoveServo_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!agrobot_interfaces__srv__MoveServo_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
agrobot_interfaces__srv__MoveServo_Event__copy(
  const agrobot_interfaces__srv__MoveServo_Event * input,
  agrobot_interfaces__srv__MoveServo_Event * output)
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
  if (!agrobot_interfaces__srv__MoveServo_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!agrobot_interfaces__srv__MoveServo_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

agrobot_interfaces__srv__MoveServo_Event *
agrobot_interfaces__srv__MoveServo_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrobot_interfaces__srv__MoveServo_Event * msg = (agrobot_interfaces__srv__MoveServo_Event *)allocator.allocate(sizeof(agrobot_interfaces__srv__MoveServo_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(agrobot_interfaces__srv__MoveServo_Event));
  bool success = agrobot_interfaces__srv__MoveServo_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
agrobot_interfaces__srv__MoveServo_Event__destroy(agrobot_interfaces__srv__MoveServo_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    agrobot_interfaces__srv__MoveServo_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
agrobot_interfaces__srv__MoveServo_Event__Sequence__init(agrobot_interfaces__srv__MoveServo_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrobot_interfaces__srv__MoveServo_Event * data = NULL;

  if (size) {
    data = (agrobot_interfaces__srv__MoveServo_Event *)allocator.zero_allocate(size, sizeof(agrobot_interfaces__srv__MoveServo_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = agrobot_interfaces__srv__MoveServo_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        agrobot_interfaces__srv__MoveServo_Event__fini(&data[i - 1]);
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
agrobot_interfaces__srv__MoveServo_Event__Sequence__fini(agrobot_interfaces__srv__MoveServo_Event__Sequence * array)
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
      agrobot_interfaces__srv__MoveServo_Event__fini(&array->data[i]);
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

agrobot_interfaces__srv__MoveServo_Event__Sequence *
agrobot_interfaces__srv__MoveServo_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  agrobot_interfaces__srv__MoveServo_Event__Sequence * array = (agrobot_interfaces__srv__MoveServo_Event__Sequence *)allocator.allocate(sizeof(agrobot_interfaces__srv__MoveServo_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = agrobot_interfaces__srv__MoveServo_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
agrobot_interfaces__srv__MoveServo_Event__Sequence__destroy(agrobot_interfaces__srv__MoveServo_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    agrobot_interfaces__srv__MoveServo_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
agrobot_interfaces__srv__MoveServo_Event__Sequence__are_equal(const agrobot_interfaces__srv__MoveServo_Event__Sequence * lhs, const agrobot_interfaces__srv__MoveServo_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!agrobot_interfaces__srv__MoveServo_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
agrobot_interfaces__srv__MoveServo_Event__Sequence__copy(
  const agrobot_interfaces__srv__MoveServo_Event__Sequence * input,
  agrobot_interfaces__srv__MoveServo_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(agrobot_interfaces__srv__MoveServo_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    agrobot_interfaces__srv__MoveServo_Event * data =
      (agrobot_interfaces__srv__MoveServo_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!agrobot_interfaces__srv__MoveServo_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          agrobot_interfaces__srv__MoveServo_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!agrobot_interfaces__srv__MoveServo_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
