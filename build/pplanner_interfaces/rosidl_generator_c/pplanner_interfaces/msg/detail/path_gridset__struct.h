// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pplanner_interfaces:msg/PathGridset.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__STRUCT_H_
#define PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "pplanner_interfaces/msg/detail/path_grid__struct.h"

/// Struct defined in msg/PathGridset in the package pplanner_interfaces.
typedef struct pplanner_interfaces__msg__PathGridset
{
  pplanner_interfaces__msg__PathGrid__Sequence path;
} pplanner_interfaces__msg__PathGridset;

// Struct for a sequence of pplanner_interfaces__msg__PathGridset.
typedef struct pplanner_interfaces__msg__PathGridset__Sequence
{
  pplanner_interfaces__msg__PathGridset * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pplanner_interfaces__msg__PathGridset__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__STRUCT_H_
