// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from basicmicro_ros2:srv/GetAvailableHomingMethods.idl
// generated code does not contain a copyright notice
#include "basicmicro_ros2/srv/detail/get_available_homing_methods__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "basicmicro_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.h"
#include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
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


using _GetAvailableHomingMethods_Request__ros_msg_type = basicmicro_ros2__srv__GetAvailableHomingMethods_Request;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_serialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_deserialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  eprosima::fastcdr::Cdr & cdr,
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request * ros_message)
{
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t get_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetAvailableHomingMethods_Request__ros_msg_type * ros_message = static_cast<const _GetAvailableHomingMethods_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t max_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
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

  // Field name: structure_needs_at_least_one_member
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = basicmicro_ros2__srv__GetAvailableHomingMethods_Request;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_serialize_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t get_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetAvailableHomingMethods_Request__ros_msg_type * ros_message = static_cast<const _GetAvailableHomingMethods_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t max_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
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
  // Field name: structure_needs_at_least_one_member
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = basicmicro_ros2__srv__GetAvailableHomingMethods_Request;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _GetAvailableHomingMethods_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Request * ros_message = static_cast<const basicmicro_ros2__srv__GetAvailableHomingMethods_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(ros_message, cdr);
}

static bool _GetAvailableHomingMethods_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request * ros_message = static_cast<basicmicro_ros2__srv__GetAvailableHomingMethods_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(cdr, ros_message);
}

static uint32_t _GetAvailableHomingMethods_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
      untyped_ros_message, 0));
}

static size_t _GetAvailableHomingMethods_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetAvailableHomingMethods_Request = {
  "basicmicro_ros2::srv",
  "GetAvailableHomingMethods_Request",
  _GetAvailableHomingMethods_Request__cdr_serialize,
  _GetAvailableHomingMethods_Request__cdr_deserialize,
  _GetAvailableHomingMethods_Request__get_serialized_size,
  _GetAvailableHomingMethods_Request__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _GetAvailableHomingMethods_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetAvailableHomingMethods_Request,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Request__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)() {
  return &_GetAvailableHomingMethods_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "basicmicro_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "rosidl_runtime_c/primitives_sequence.h"  // acts_as_limit, auto_zeros_encoder
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // acts_as_limit, auto_zeros_encoder
#include "rosidl_runtime_c/string.h"  // allowed_directions, available_methods, controller_type, method_descriptions
#include "rosidl_runtime_c/string_functions.h"  // allowed_directions, available_methods, controller_type, method_descriptions

// forward declare type support functions


using _GetAvailableHomingMethods_Response__ros_msg_type = basicmicro_ros2__srv__GetAvailableHomingMethods_Response;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_serialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: controller_type
  {
    const rosidl_runtime_c__String * str = &ros_message->controller_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: available_methods
  {
    size_t size = ros_message->available_methods.size;
    auto array_ptr = ros_message->available_methods.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: method_descriptions
  {
    size_t size = ros_message->method_descriptions.size;
    auto array_ptr = ros_message->method_descriptions.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: allowed_directions
  {
    size_t size = ros_message->allowed_directions.size;
    auto array_ptr = ros_message->allowed_directions.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: auto_zeros_encoder
  {
    size_t size = ros_message->auto_zeros_encoder.size;
    auto array_ptr = ros_message->auto_zeros_encoder.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: acts_as_limit
  {
    size_t size = ros_message->acts_as_limit.size;
    auto array_ptr = ros_message->acts_as_limit.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_deserialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  eprosima::fastcdr::Cdr & cdr,
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response * ros_message)
{
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: controller_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->controller_type.data) {
      rosidl_runtime_c__String__init(&ros_message->controller_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->controller_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'controller_type'\n");
      return false;
    }
  }

  // Field name: available_methods
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->available_methods.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->available_methods);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->available_methods, size)) {
      fprintf(stderr, "failed to create array for field 'available_methods'");
      return false;
    }
    auto array_ptr = ros_message->available_methods.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'available_methods'\n");
        return false;
      }
    }
  }

  // Field name: method_descriptions
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->method_descriptions.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->method_descriptions);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->method_descriptions, size)) {
      fprintf(stderr, "failed to create array for field 'method_descriptions'");
      return false;
    }
    auto array_ptr = ros_message->method_descriptions.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'method_descriptions'\n");
        return false;
      }
    }
  }

  // Field name: allowed_directions
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->allowed_directions.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->allowed_directions);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->allowed_directions, size)) {
      fprintf(stderr, "failed to create array for field 'allowed_directions'");
      return false;
    }
    auto array_ptr = ros_message->allowed_directions.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'allowed_directions'\n");
        return false;
      }
    }
  }

  // Field name: auto_zeros_encoder
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->auto_zeros_encoder.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->auto_zeros_encoder);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->auto_zeros_encoder, size)) {
      fprintf(stderr, "failed to create array for field 'auto_zeros_encoder'");
      return false;
    }
    auto array_ptr = ros_message->auto_zeros_encoder.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: acts_as_limit
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->acts_as_limit.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->acts_as_limit);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->acts_as_limit, size)) {
      fprintf(stderr, "failed to create array for field 'acts_as_limit'");
      return false;
    }
    auto array_ptr = ros_message->acts_as_limit.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t get_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetAvailableHomingMethods_Response__ros_msg_type * ros_message = static_cast<const _GetAvailableHomingMethods_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: controller_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->controller_type.size + 1);

  // Field name: available_methods
  {
    size_t array_size = ros_message->available_methods.size;
    auto array_ptr = ros_message->available_methods.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  // Field name: method_descriptions
  {
    size_t array_size = ros_message->method_descriptions.size;
    auto array_ptr = ros_message->method_descriptions.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  // Field name: allowed_directions
  {
    size_t array_size = ros_message->allowed_directions.size;
    auto array_ptr = ros_message->allowed_directions.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  // Field name: auto_zeros_encoder
  {
    size_t array_size = ros_message->auto_zeros_encoder.size;
    auto array_ptr = ros_message->auto_zeros_encoder.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: acts_as_limit
  {
    size_t array_size = ros_message->acts_as_limit.size;
    auto array_ptr = ros_message->acts_as_limit.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t max_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
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

  // Field name: success
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: controller_type
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: available_methods
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: method_descriptions
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: allowed_directions
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: auto_zeros_encoder
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: acts_as_limit
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = basicmicro_ros2__srv__GetAvailableHomingMethods_Response;
    is_plain =
      (
      offsetof(DataType, acts_as_limit) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_serialize_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: controller_type
  {
    const rosidl_runtime_c__String * str = &ros_message->controller_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: available_methods
  {
    size_t size = ros_message->available_methods.size;
    auto array_ptr = ros_message->available_methods.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: method_descriptions
  {
    size_t size = ros_message->method_descriptions.size;
    auto array_ptr = ros_message->method_descriptions.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: allowed_directions
  {
    size_t size = ros_message->allowed_directions.size;
    auto array_ptr = ros_message->allowed_directions.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: auto_zeros_encoder
  {
    size_t size = ros_message->auto_zeros_encoder.size;
    auto array_ptr = ros_message->auto_zeros_encoder.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: acts_as_limit
  {
    size_t size = ros_message->acts_as_limit.size;
    auto array_ptr = ros_message->acts_as_limit.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t get_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetAvailableHomingMethods_Response__ros_msg_type * ros_message = static_cast<const _GetAvailableHomingMethods_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: controller_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->controller_type.size + 1);

  // Field name: available_methods
  {
    size_t array_size = ros_message->available_methods.size;
    auto array_ptr = ros_message->available_methods.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  // Field name: method_descriptions
  {
    size_t array_size = ros_message->method_descriptions.size;
    auto array_ptr = ros_message->method_descriptions.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  // Field name: allowed_directions
  {
    size_t array_size = ros_message->allowed_directions.size;
    auto array_ptr = ros_message->allowed_directions.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  // Field name: auto_zeros_encoder
  {
    size_t array_size = ros_message->auto_zeros_encoder.size;
    auto array_ptr = ros_message->auto_zeros_encoder.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: acts_as_limit
  {
    size_t array_size = ros_message->acts_as_limit.size;
    auto array_ptr = ros_message->acts_as_limit.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t max_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
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
  // Field name: success
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: controller_type
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: available_methods
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: method_descriptions
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: allowed_directions
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: auto_zeros_encoder
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: acts_as_limit
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = basicmicro_ros2__srv__GetAvailableHomingMethods_Response;
    is_plain =
      (
      offsetof(DataType, acts_as_limit) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _GetAvailableHomingMethods_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Response * ros_message = static_cast<const basicmicro_ros2__srv__GetAvailableHomingMethods_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(ros_message, cdr);
}

static bool _GetAvailableHomingMethods_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response * ros_message = static_cast<basicmicro_ros2__srv__GetAvailableHomingMethods_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(cdr, ros_message);
}

static uint32_t _GetAvailableHomingMethods_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
      untyped_ros_message, 0));
}

static size_t _GetAvailableHomingMethods_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetAvailableHomingMethods_Response = {
  "basicmicro_ros2::srv",
  "GetAvailableHomingMethods_Response",
  _GetAvailableHomingMethods_Response__cdr_serialize,
  _GetAvailableHomingMethods_Response__cdr_deserialize,
  _GetAvailableHomingMethods_Response__get_serialized_size,
  _GetAvailableHomingMethods_Response__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _GetAvailableHomingMethods_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetAvailableHomingMethods_Response,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Response__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)() {
  return &_GetAvailableHomingMethods_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "basicmicro_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__struct.h"
// already included above
// #include "basicmicro_ros2/srv/detail/get_available_homing_methods__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "service_msgs/msg/detail/service_event_info__functions.h"  // info

// forward declare type support functions

bool cdr_serialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  eprosima::fastcdr::Cdr & cdr,
  basicmicro_ros2__srv__GetAvailableHomingMethods_Request * ros_message);

size_t get_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)();

bool cdr_serialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  eprosima::fastcdr::Cdr & cdr,
  basicmicro_ros2__srv__GetAvailableHomingMethods_Response * ros_message);

size_t get_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_basicmicro_ros2
bool cdr_serialize_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_basicmicro_ros2
bool cdr_deserialize_service_msgs__msg__ServiceEventInfo(
  eprosima::fastcdr::Cdr & cdr,
  service_msgs__msg__ServiceEventInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_basicmicro_ros2
size_t get_serialized_size_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_basicmicro_ros2
size_t max_serialized_size_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_basicmicro_ros2
bool cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_basicmicro_ros2
size_t get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_basicmicro_ros2
size_t max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_basicmicro_ros2
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, service_msgs, msg, ServiceEventInfo)();


using _GetAvailableHomingMethods_Event__ros_msg_type = basicmicro_ros2__srv__GetAvailableHomingMethods_Event;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_serialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_deserialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(
  eprosima::fastcdr::Cdr & cdr,
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event * ros_message)
{
  // Field name: info
  {
    cdr_deserialize_service_msgs__msg__ServiceEventInfo(cdr, &ros_message->info);
  }

  // Field name: request
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->request.data) {
      basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence__fini(&ros_message->request);
    }
    if (!basicmicro_ros2__srv__GetAvailableHomingMethods_Request__Sequence__init(&ros_message->request, size)) {
      fprintf(stderr, "failed to create array for field 'request'");
      return false;
    }
    auto array_ptr = ros_message->request.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(cdr, &array_ptr[i]);
    }
  }

  // Field name: response
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->response.data) {
      basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence__fini(&ros_message->response);
    }
    if (!basicmicro_ros2__srv__GetAvailableHomingMethods_Response__Sequence__init(&ros_message->response, size)) {
      fprintf(stderr, "failed to create array for field 'response'");
      return false;
    }
    auto array_ptr = ros_message->response.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t get_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetAvailableHomingMethods_Event__ros_msg_type * ros_message = static_cast<const _GetAvailableHomingMethods_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t max_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(
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

  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = basicmicro_ros2__srv__GetAvailableHomingMethods_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
bool cdr_serialize_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t get_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetAvailableHomingMethods_Event__ros_msg_type * ros_message = static_cast<const _GetAvailableHomingMethods_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_basicmicro_ros2
size_t max_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(
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
  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_basicmicro_ros2__srv__GetAvailableHomingMethods_Response(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = basicmicro_ros2__srv__GetAvailableHomingMethods_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _GetAvailableHomingMethods_Event__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const basicmicro_ros2__srv__GetAvailableHomingMethods_Event * ros_message = static_cast<const basicmicro_ros2__srv__GetAvailableHomingMethods_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(ros_message, cdr);
}

static bool _GetAvailableHomingMethods_Event__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  basicmicro_ros2__srv__GetAvailableHomingMethods_Event * ros_message = static_cast<basicmicro_ros2__srv__GetAvailableHomingMethods_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(cdr, ros_message);
}

static uint32_t _GetAvailableHomingMethods_Event__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(
      untyped_ros_message, 0));
}

static size_t _GetAvailableHomingMethods_Event__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_basicmicro_ros2__srv__GetAvailableHomingMethods_Event(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetAvailableHomingMethods_Event = {
  "basicmicro_ros2::srv",
  "GetAvailableHomingMethods_Event",
  _GetAvailableHomingMethods_Event__cdr_serialize,
  _GetAvailableHomingMethods_Event__cdr_deserialize,
  _GetAvailableHomingMethods_Event__get_serialized_size,
  _GetAvailableHomingMethods_Event__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _GetAvailableHomingMethods_Event__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetAvailableHomingMethods_Event,
  get_message_typesupport_handle_function,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods_Event__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Event)() {
  return &_GetAvailableHomingMethods_Event__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "basicmicro_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "basicmicro_ros2/srv/get_available_homing_methods.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GetAvailableHomingMethods__callbacks = {
  "basicmicro_ros2::srv",
  "GetAvailableHomingMethods",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, GetAvailableHomingMethods_Response)(),
};

static rosidl_service_type_support_t GetAvailableHomingMethods__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GetAvailableHomingMethods__callbacks,
  get_service_typesupport_handle_function,
  &_GetAvailableHomingMethods_Request__type_support,
  &_GetAvailableHomingMethods_Response__type_support,
  &_GetAvailableHomingMethods_Event__type_support,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    GetAvailableHomingMethods
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    basicmicro_ros2,
    srv,
    GetAvailableHomingMethods
  ),
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_hash,
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_description,
  &basicmicro_ros2__srv__GetAvailableHomingMethods__get_type_description_sources,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, basicmicro_ros2, srv, GetAvailableHomingMethods)() {
  return &GetAvailableHomingMethods__handle;
}

#if defined(__cplusplus)
}
#endif
