// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from lsm10_v2:msg/Difop.idl
// generated code does not contain a copyright notice

#ifndef LSM10_V2__MSG__DETAIL__DIFOP__FUNCTIONS_H_
#define LSM10_V2__MSG__DETAIL__DIFOP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "lsm10_v2/msg/rosidl_generator_c__visibility_control.h"

#include "lsm10_v2/msg/detail/difop__struct.h"

/// Initialize msg/Difop message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lsm10_v2__msg__Difop
 * )) before or use
 * lsm10_v2__msg__Difop__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
bool
lsm10_v2__msg__Difop__init(lsm10_v2__msg__Difop * msg);

/// Finalize msg/Difop message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
void
lsm10_v2__msg__Difop__fini(lsm10_v2__msg__Difop * msg);

/// Create msg/Difop message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lsm10_v2__msg__Difop__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
lsm10_v2__msg__Difop *
lsm10_v2__msg__Difop__create();

/// Destroy msg/Difop message.
/**
 * It calls
 * lsm10_v2__msg__Difop__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
void
lsm10_v2__msg__Difop__destroy(lsm10_v2__msg__Difop * msg);

/// Check for msg/Difop message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
bool
lsm10_v2__msg__Difop__are_equal(const lsm10_v2__msg__Difop * lhs, const lsm10_v2__msg__Difop * rhs);

/// Copy a msg/Difop message.
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
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
bool
lsm10_v2__msg__Difop__copy(
  const lsm10_v2__msg__Difop * input,
  lsm10_v2__msg__Difop * output);

/// Initialize array of msg/Difop messages.
/**
 * It allocates the memory for the number of elements and calls
 * lsm10_v2__msg__Difop__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
bool
lsm10_v2__msg__Difop__Sequence__init(lsm10_v2__msg__Difop__Sequence * array, size_t size);

/// Finalize array of msg/Difop messages.
/**
 * It calls
 * lsm10_v2__msg__Difop__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
void
lsm10_v2__msg__Difop__Sequence__fini(lsm10_v2__msg__Difop__Sequence * array);

/// Create array of msg/Difop messages.
/**
 * It allocates the memory for the array and calls
 * lsm10_v2__msg__Difop__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
lsm10_v2__msg__Difop__Sequence *
lsm10_v2__msg__Difop__Sequence__create(size_t size);

/// Destroy array of msg/Difop messages.
/**
 * It calls
 * lsm10_v2__msg__Difop__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
void
lsm10_v2__msg__Difop__Sequence__destroy(lsm10_v2__msg__Difop__Sequence * array);

/// Check for msg/Difop message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
bool
lsm10_v2__msg__Difop__Sequence__are_equal(const lsm10_v2__msg__Difop__Sequence * lhs, const lsm10_v2__msg__Difop__Sequence * rhs);

/// Copy an array of msg/Difop messages.
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
ROSIDL_GENERATOR_C_PUBLIC_lsm10_v2
bool
lsm10_v2__msg__Difop__Sequence__copy(
  const lsm10_v2__msg__Difop__Sequence * input,
  lsm10_v2__msg__Difop__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LSM10_V2__MSG__DETAIL__DIFOP__FUNCTIONS_H_
