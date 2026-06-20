// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from basicmicro_ros2:msg/PositionPoint.idl
// generated code does not contain a copyright notice
#include "basicmicro_ros2/msg/detail/position_point__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "basicmicro_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "basicmicro_ros2/msg/detail/position_point__struct.h"
#include "basicmicro_ros2/msg/detail/position_point__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _PositionPoint__ros_msg_type = basicmicro_ros2__msg__PositionPoint;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_serialize_basicmicro_ros2__msg__PositionPoint(
  const basicmicro_ros2__msg__PositionPoint * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: left_position
  {
    cdr << ros_message->left_position;
  }

  // Field name: right_position
  {
    cdr << ros_message->right_position;
  }

  // Field name: max_speed
  {
    cdr << ros_message->max_speed;
  }

  // Field name: acceleration
  {
    cdr << ros_message->acceleration;
  }

  // Field name: deceleration
  {
    cdr << ros_message->deceleration;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_deserialize_basicmicro_ros2__msg__PositionPoint(
  eprosima::fastcdr::Cdr & cdr,
  basicmicro_ros2__msg__PositionPoint * ros_message)
{
  // Field name: left_position
  {
    cdr >> ros_message->left_position;
  }

  // Field name: right_position
  {
    cdr >> ros_message->right_position;
  }

  // Field name: max_speed
  {
    cdr >> ros_message->max_speed;
  }

  // Field name: acceleration
  {
    cdr >> ros_message->acceleration;
  }

  // Field name: deceleration
  {
    cdr >> ros_message->deceleration;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t get_serialized_size_basicmicro_ros2__msg__PositionPoint(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PositionPoint__ros_msg_type * ros_message = static_cast<const _PositionPoint__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: left_position
  {
    size_t item_size = sizeof(ros_message->left_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: right_position
  {
    size_t item_size = sizeof(ros_message->right_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_speed
  {
    size_t item_size = sizeof(ros_message->max_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: acceleration
  {
    size_t item_size = sizeof(ros_message->acceleration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: deceleration
  {
    size_t item_size = sizeof(ros_message->deceleration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t max_serialized_size_basicmicro_ros2__msg__PositionPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: left_position
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: right_position
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: max_speed
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: acceleration
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: deceleration
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = basicmicro_ros2__msg__PositionPoint;
    is_plain =
      (
      offsetof(DataType, deceleration) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_serialize_key_basicmicro_ros2__msg__PositionPoint(
  const basicmicro_ros2__msg__PositionPoint * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: left_position
  {
    cdr << ros_message->left_position;
  }

  // Field name: right_position
  {
    cdr << ros_message->right_position;
  }

  // Field name: max_speed
  {
    cdr << ros_message->max_speed;
  }

  // Field name: acceleration
  {
    cdr << ros_message->acceleration;
  }

  // Field name: deceleration
  {
    cdr << ros_message->deceleration;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t get_serialized_size_key_basicmicro_ros2__msg__PositionPoint(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PositionPoint__ros_msg_type * ros_message = static_cast<const _PositionPoint__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: left_position
  {
    size_t item_size = sizeof(ros_message->left_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: right_position
  {
    size_t item_size = sizeof(ros_message->right_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_speed
  {
    size_t item_size = sizeof(ros_message->max_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: acceleration
  {
    size_t item_size = sizeof(ros_message->acceleration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: deceleration
  {
    size_t item_size = sizeof(ros_message->deceleration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t max_serialized_size_key_basicmicro_ros2__msg__PositionPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: left_position
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: right_position
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: max_speed
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: acceleration
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: deceleration
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = basicmicro_ros2__msg__PositionPoint;
    is_plain =
      (
      offsetof(DataType, deceleration) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _PositionPoint__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const basicmicro_ros2__msg__PositionPoint * ros_message = static_cast<const basicmicro_ros2__msg__PositionPoint *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_basicmicro_ros2__msg__PositionPoint(ros_message, cdr);
}

static bool _PositionPoint__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  basicmicro_ros2__msg__PositionPoint * ros_message = static_cast<basicmicro_ros2__msg__PositionPoint *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_basicmicro_ros2__msg__PositionPoint(cdr, ros_message);
}

static uint32_t _PositionPoint__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_basicmicro_ros2__msg__PositionPoint(
      untyped_ros_message, 0));
}

static size_t _PositionPoint__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_basicmicro_ros2__msg__PositionPoint(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PositionPoint = {
  "basicmicro_ros2::msg",
  "PositionPoint",
  _PositionPoint__cdr_serialize,
  _PositionPoint__cdr_deserialize,
  _PositionPoint__get_serialized_size,
  _PositionPoint__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _PositionPoint__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PositionPoint,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__msg__PositionPoint__get_type_hash,
  &basicmicro_ros2__msg__PositionPoint__get_type_description,
  &basicmicro_ros2__msg__PositionPoint__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, msg, PositionPoint)() {
  return &_PositionPoint__type_support;
}

#if defined(__cplusplus)
}
#endif
