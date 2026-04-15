// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from agrobot_interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "agrobot_interfaces/srv/move_servo.h"


#ifndef AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__FUNCTIONS_H_
#define AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "agrobot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "agrobot_interfaces/srv/detail/move_servo__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_type_hash_t *
agrobot_interfaces__srv__MoveServo__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
agrobot_interfaces__srv__MoveServo__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeSource *
agrobot_interfaces__srv__MoveServo__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
agrobot_interfaces__srv__MoveServo__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize srv/MoveServo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * agrobot_interfaces__srv__MoveServo_Request
 * )) before or use
 * agrobot_interfaces__srv__MoveServo_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Request__init(agrobot_interfaces__srv__MoveServo_Request * msg);

/// Finalize srv/MoveServo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Request__fini(agrobot_interfaces__srv__MoveServo_Request * msg);

/// Create srv/MoveServo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * agrobot_interfaces__srv__MoveServo_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
agrobot_interfaces__srv__MoveServo_Request *
agrobot_interfaces__srv__MoveServo_Request__create(void);

/// Destroy srv/MoveServo message.
/**
 * It calls
 * agrobot_interfaces__srv__MoveServo_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Request__destroy(agrobot_interfaces__srv__MoveServo_Request * msg);

/// Check for srv/MoveServo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Request__are_equal(const agrobot_interfaces__srv__MoveServo_Request * lhs, const agrobot_interfaces__srv__MoveServo_Request * rhs);

/// Copy a srv/MoveServo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Request__copy(
  const agrobot_interfaces__srv__MoveServo_Request * input,
  agrobot_interfaces__srv__MoveServo_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_type_hash_t *
agrobot_interfaces__srv__MoveServo_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
agrobot_interfaces__srv__MoveServo_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeSource *
agrobot_interfaces__srv__MoveServo_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
agrobot_interfaces__srv__MoveServo_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/MoveServo messages.
/**
 * It allocates the memory for the number of elements and calls
 * agrobot_interfaces__srv__MoveServo_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Request__Sequence__init(agrobot_interfaces__srv__MoveServo_Request__Sequence * array, size_t size);

/// Finalize array of srv/MoveServo messages.
/**
 * It calls
 * agrobot_interfaces__srv__MoveServo_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Request__Sequence__fini(agrobot_interfaces__srv__MoveServo_Request__Sequence * array);

/// Create array of srv/MoveServo messages.
/**
 * It allocates the memory for the array and calls
 * agrobot_interfaces__srv__MoveServo_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
agrobot_interfaces__srv__MoveServo_Request__Sequence *
agrobot_interfaces__srv__MoveServo_Request__Sequence__create(size_t size);

/// Destroy array of srv/MoveServo messages.
/**
 * It calls
 * agrobot_interfaces__srv__MoveServo_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Request__Sequence__destroy(agrobot_interfaces__srv__MoveServo_Request__Sequence * array);

/// Check for srv/MoveServo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Request__Sequence__are_equal(const agrobot_interfaces__srv__MoveServo_Request__Sequence * lhs, const agrobot_interfaces__srv__MoveServo_Request__Sequence * rhs);

/// Copy an array of srv/MoveServo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Request__Sequence__copy(
  const agrobot_interfaces__srv__MoveServo_Request__Sequence * input,
  agrobot_interfaces__srv__MoveServo_Request__Sequence * output);

/// Initialize srv/MoveServo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * agrobot_interfaces__srv__MoveServo_Response
 * )) before or use
 * agrobot_interfaces__srv__MoveServo_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Response__init(agrobot_interfaces__srv__MoveServo_Response * msg);

/// Finalize srv/MoveServo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Response__fini(agrobot_interfaces__srv__MoveServo_Response * msg);

/// Create srv/MoveServo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * agrobot_interfaces__srv__MoveServo_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
agrobot_interfaces__srv__MoveServo_Response *
agrobot_interfaces__srv__MoveServo_Response__create(void);

/// Destroy srv/MoveServo message.
/**
 * It calls
 * agrobot_interfaces__srv__MoveServo_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Response__destroy(agrobot_interfaces__srv__MoveServo_Response * msg);

/// Check for srv/MoveServo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Response__are_equal(const agrobot_interfaces__srv__MoveServo_Response * lhs, const agrobot_interfaces__srv__MoveServo_Response * rhs);

/// Copy a srv/MoveServo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Response__copy(
  const agrobot_interfaces__srv__MoveServo_Response * input,
  agrobot_interfaces__srv__MoveServo_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_type_hash_t *
agrobot_interfaces__srv__MoveServo_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
agrobot_interfaces__srv__MoveServo_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeSource *
agrobot_interfaces__srv__MoveServo_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
agrobot_interfaces__srv__MoveServo_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/MoveServo messages.
/**
 * It allocates the memory for the number of elements and calls
 * agrobot_interfaces__srv__MoveServo_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Response__Sequence__init(agrobot_interfaces__srv__MoveServo_Response__Sequence * array, size_t size);

/// Finalize array of srv/MoveServo messages.
/**
 * It calls
 * agrobot_interfaces__srv__MoveServo_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Response__Sequence__fini(agrobot_interfaces__srv__MoveServo_Response__Sequence * array);

/// Create array of srv/MoveServo messages.
/**
 * It allocates the memory for the array and calls
 * agrobot_interfaces__srv__MoveServo_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
agrobot_interfaces__srv__MoveServo_Response__Sequence *
agrobot_interfaces__srv__MoveServo_Response__Sequence__create(size_t size);

/// Destroy array of srv/MoveServo messages.
/**
 * It calls
 * agrobot_interfaces__srv__MoveServo_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Response__Sequence__destroy(agrobot_interfaces__srv__MoveServo_Response__Sequence * array);

/// Check for srv/MoveServo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Response__Sequence__are_equal(const agrobot_interfaces__srv__MoveServo_Response__Sequence * lhs, const agrobot_interfaces__srv__MoveServo_Response__Sequence * rhs);

/// Copy an array of srv/MoveServo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Response__Sequence__copy(
  const agrobot_interfaces__srv__MoveServo_Response__Sequence * input,
  agrobot_interfaces__srv__MoveServo_Response__Sequence * output);

/// Initialize srv/MoveServo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * agrobot_interfaces__srv__MoveServo_Event
 * )) before or use
 * agrobot_interfaces__srv__MoveServo_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Event__init(agrobot_interfaces__srv__MoveServo_Event * msg);

/// Finalize srv/MoveServo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Event__fini(agrobot_interfaces__srv__MoveServo_Event * msg);

/// Create srv/MoveServo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * agrobot_interfaces__srv__MoveServo_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
agrobot_interfaces__srv__MoveServo_Event *
agrobot_interfaces__srv__MoveServo_Event__create(void);

/// Destroy srv/MoveServo message.
/**
 * It calls
 * agrobot_interfaces__srv__MoveServo_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Event__destroy(agrobot_interfaces__srv__MoveServo_Event * msg);

/// Check for srv/MoveServo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Event__are_equal(const agrobot_interfaces__srv__MoveServo_Event * lhs, const agrobot_interfaces__srv__MoveServo_Event * rhs);

/// Copy a srv/MoveServo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Event__copy(
  const agrobot_interfaces__srv__MoveServo_Event * input,
  agrobot_interfaces__srv__MoveServo_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_type_hash_t *
agrobot_interfaces__srv__MoveServo_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
agrobot_interfaces__srv__MoveServo_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeSource *
agrobot_interfaces__srv__MoveServo_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
agrobot_interfaces__srv__MoveServo_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/MoveServo messages.
/**
 * It allocates the memory for the number of elements and calls
 * agrobot_interfaces__srv__MoveServo_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Event__Sequence__init(agrobot_interfaces__srv__MoveServo_Event__Sequence * array, size_t size);

/// Finalize array of srv/MoveServo messages.
/**
 * It calls
 * agrobot_interfaces__srv__MoveServo_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Event__Sequence__fini(agrobot_interfaces__srv__MoveServo_Event__Sequence * array);

/// Create array of srv/MoveServo messages.
/**
 * It allocates the memory for the array and calls
 * agrobot_interfaces__srv__MoveServo_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
agrobot_interfaces__srv__MoveServo_Event__Sequence *
agrobot_interfaces__srv__MoveServo_Event__Sequence__create(size_t size);

/// Destroy array of srv/MoveServo messages.
/**
 * It calls
 * agrobot_interfaces__srv__MoveServo_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
void
agrobot_interfaces__srv__MoveServo_Event__Sequence__destroy(agrobot_interfaces__srv__MoveServo_Event__Sequence * array);

/// Check for srv/MoveServo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Event__Sequence__are_equal(const agrobot_interfaces__srv__MoveServo_Event__Sequence * lhs, const agrobot_interfaces__srv__MoveServo_Event__Sequence * rhs);

/// Copy an array of srv/MoveServo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_agrobot_interfaces
bool
agrobot_interfaces__srv__MoveServo_Event__Sequence__copy(
  const agrobot_interfaces__srv__MoveServo_Event__Sequence * input,
  agrobot_interfaces__srv__MoveServo_Event__Sequence * output);
#ifdef __cplusplus
}
#endif

#endif  // AGROBOT_INTERFACES__SRV__DETAIL__MOVE_SERVO__FUNCTIONS_H_
