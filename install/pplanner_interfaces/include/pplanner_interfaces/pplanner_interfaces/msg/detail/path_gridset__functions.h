// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pplanner_interfaces:msg/PathGridset.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__FUNCTIONS_H_
#define PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pplanner_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "pplanner_interfaces/msg/detail/path_gridset__struct.h"

/// Initialize msg/PathGridset message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pplanner_interfaces__msg__PathGridset
 * )) before or use
 * pplanner_interfaces__msg__PathGridset__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
bool
pplanner_interfaces__msg__PathGridset__init(pplanner_interfaces__msg__PathGridset * msg);

/// Finalize msg/PathGridset message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
void
pplanner_interfaces__msg__PathGridset__fini(pplanner_interfaces__msg__PathGridset * msg);

/// Create msg/PathGridset message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pplanner_interfaces__msg__PathGridset__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
pplanner_interfaces__msg__PathGridset *
pplanner_interfaces__msg__PathGridset__create();

/// Destroy msg/PathGridset message.
/**
 * It calls
 * pplanner_interfaces__msg__PathGridset__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
void
pplanner_interfaces__msg__PathGridset__destroy(pplanner_interfaces__msg__PathGridset * msg);

/// Check for msg/PathGridset message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
bool
pplanner_interfaces__msg__PathGridset__are_equal(const pplanner_interfaces__msg__PathGridset * lhs, const pplanner_interfaces__msg__PathGridset * rhs);

/// Copy a msg/PathGridset message.
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
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
bool
pplanner_interfaces__msg__PathGridset__copy(
  const pplanner_interfaces__msg__PathGridset * input,
  pplanner_interfaces__msg__PathGridset * output);

/// Initialize array of msg/PathGridset messages.
/**
 * It allocates the memory for the number of elements and calls
 * pplanner_interfaces__msg__PathGridset__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
bool
pplanner_interfaces__msg__PathGridset__Sequence__init(pplanner_interfaces__msg__PathGridset__Sequence * array, size_t size);

/// Finalize array of msg/PathGridset messages.
/**
 * It calls
 * pplanner_interfaces__msg__PathGridset__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
void
pplanner_interfaces__msg__PathGridset__Sequence__fini(pplanner_interfaces__msg__PathGridset__Sequence * array);

/// Create array of msg/PathGridset messages.
/**
 * It allocates the memory for the array and calls
 * pplanner_interfaces__msg__PathGridset__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
pplanner_interfaces__msg__PathGridset__Sequence *
pplanner_interfaces__msg__PathGridset__Sequence__create(size_t size);

/// Destroy array of msg/PathGridset messages.
/**
 * It calls
 * pplanner_interfaces__msg__PathGridset__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
void
pplanner_interfaces__msg__PathGridset__Sequence__destroy(pplanner_interfaces__msg__PathGridset__Sequence * array);

/// Check for msg/PathGridset message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
bool
pplanner_interfaces__msg__PathGridset__Sequence__are_equal(const pplanner_interfaces__msg__PathGridset__Sequence * lhs, const pplanner_interfaces__msg__PathGridset__Sequence * rhs);

/// Copy an array of msg/PathGridset messages.
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
ROSIDL_GENERATOR_C_PUBLIC_pplanner_interfaces
bool
pplanner_interfaces__msg__PathGridset__Sequence__copy(
  const pplanner_interfaces__msg__PathGridset__Sequence * input,
  pplanner_interfaces__msg__PathGridset__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__FUNCTIONS_H_
