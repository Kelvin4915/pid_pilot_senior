// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pplanner_interfaces:msg/PathGrid.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__STRUCT_H_
#define PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PathGrid in the package pplanner_interfaces.
typedef struct pplanner_interfaces__msg__PathGrid
{
  int64_t row;
  int64_t col;
} pplanner_interfaces__msg__PathGrid;

// Struct for a sequence of pplanner_interfaces__msg__PathGrid.
typedef struct pplanner_interfaces__msg__PathGrid__Sequence
{
  pplanner_interfaces__msg__PathGrid * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pplanner_interfaces__msg__PathGrid__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__STRUCT_H_
