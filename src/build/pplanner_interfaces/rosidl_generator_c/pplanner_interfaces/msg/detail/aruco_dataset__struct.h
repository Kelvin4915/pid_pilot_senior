// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pplanner_interfaces:msg/ArucoDataset.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__STRUCT_H_
#define PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'dataset'
#include "pplanner_interfaces/msg/detail/aruco_data__struct.h"

/// Struct defined in msg/ArucoDataset in the package pplanner_interfaces.
typedef struct pplanner_interfaces__msg__ArucoDataset
{
  pplanner_interfaces__msg__ArucoData__Sequence dataset;
} pplanner_interfaces__msg__ArucoDataset;

// Struct for a sequence of pplanner_interfaces__msg__ArucoDataset.
typedef struct pplanner_interfaces__msg__ArucoDataset__Sequence
{
  pplanner_interfaces__msg__ArucoDataset * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pplanner_interfaces__msg__ArucoDataset__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__STRUCT_H_
