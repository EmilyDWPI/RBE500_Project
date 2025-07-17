// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from inverse_kin:msg/InvKin.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__MSG__DETAIL__INV_KIN__FUNCTIONS_H_
#define INVERSE_KIN__MSG__DETAIL__INV_KIN__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "inverse_kin/msg/rosidl_generator_c__visibility_control.h"

#include "inverse_kin/msg/detail/inv_kin__struct.h"

/// Initialize msg/InvKin message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * inverse_kin__msg__InvKin
 * )) before or use
 * inverse_kin__msg__InvKin__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
bool
inverse_kin__msg__InvKin__init(inverse_kin__msg__InvKin * msg);

/// Finalize msg/InvKin message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
void
inverse_kin__msg__InvKin__fini(inverse_kin__msg__InvKin * msg);

/// Create msg/InvKin message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * inverse_kin__msg__InvKin__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
inverse_kin__msg__InvKin *
inverse_kin__msg__InvKin__create();

/// Destroy msg/InvKin message.
/**
 * It calls
 * inverse_kin__msg__InvKin__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
void
inverse_kin__msg__InvKin__destroy(inverse_kin__msg__InvKin * msg);

/// Check for msg/InvKin message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
bool
inverse_kin__msg__InvKin__are_equal(const inverse_kin__msg__InvKin * lhs, const inverse_kin__msg__InvKin * rhs);

/// Copy a msg/InvKin message.
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
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
bool
inverse_kin__msg__InvKin__copy(
  const inverse_kin__msg__InvKin * input,
  inverse_kin__msg__InvKin * output);

/// Initialize array of msg/InvKin messages.
/**
 * It allocates the memory for the number of elements and calls
 * inverse_kin__msg__InvKin__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
bool
inverse_kin__msg__InvKin__Sequence__init(inverse_kin__msg__InvKin__Sequence * array, size_t size);

/// Finalize array of msg/InvKin messages.
/**
 * It calls
 * inverse_kin__msg__InvKin__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
void
inverse_kin__msg__InvKin__Sequence__fini(inverse_kin__msg__InvKin__Sequence * array);

/// Create array of msg/InvKin messages.
/**
 * It allocates the memory for the array and calls
 * inverse_kin__msg__InvKin__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
inverse_kin__msg__InvKin__Sequence *
inverse_kin__msg__InvKin__Sequence__create(size_t size);

/// Destroy array of msg/InvKin messages.
/**
 * It calls
 * inverse_kin__msg__InvKin__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
void
inverse_kin__msg__InvKin__Sequence__destroy(inverse_kin__msg__InvKin__Sequence * array);

/// Check for msg/InvKin message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
bool
inverse_kin__msg__InvKin__Sequence__are_equal(const inverse_kin__msg__InvKin__Sequence * lhs, const inverse_kin__msg__InvKin__Sequence * rhs);

/// Copy an array of msg/InvKin messages.
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
ROSIDL_GENERATOR_C_PUBLIC_inverse_kin
bool
inverse_kin__msg__InvKin__Sequence__copy(
  const inverse_kin__msg__InvKin__Sequence * input,
  inverse_kin__msg__InvKin__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // INVERSE_KIN__MSG__DETAIL__INV_KIN__FUNCTIONS_H_
