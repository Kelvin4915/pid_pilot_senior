// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pplanner_interfaces:msg/ArucoData.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__STRUCT_H_
#define PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ArucoData in the package pplanner_interfaces.
typedef struct pplanner_interfaces__msg__ArucoData
{
  int64_t id_data;
  int64_t x_data;
  int64_t y_data;
  double orientation_data;
  rosidl_runtime_c__String status;
} pplanner_interfaces__msg__ArucoData;

// Struct for a sequence of pplanner_interfaces__msg__ArucoData.
typedef struct pplanner_interfaces__msg__ArucoData__Sequence
{
  pplanner_interfaces__msg__ArucoData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pplanner_interfaces__msg__ArucoData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__STRUCT_H_
