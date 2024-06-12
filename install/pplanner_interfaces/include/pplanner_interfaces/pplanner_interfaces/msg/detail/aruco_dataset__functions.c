// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pplanner_interfaces:msg/ArucoDataset.idl
// generated code does not contain a copyright notice
#include "pplanner_interfaces/msg/detail/aruco_dataset__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `dataset`
#include "pplanner_interfaces/msg/detail/aruco_data__functions.h"

bool
pplanner_interfaces__msg__ArucoDataset__init(pplanner_interfaces__msg__ArucoDataset * msg)
{
  if (!msg) {
    return false;
  }
  // dataset
  if (!pplanner_interfaces__msg__ArucoData__Sequence__init(&msg->dataset, 0)) {
    pplanner_interfaces__msg__ArucoDataset__fini(msg);
    return false;
  }
  return true;
}

void
pplanner_interfaces__msg__ArucoDataset__fini(pplanner_interfaces__msg__ArucoDataset * msg)
{
  if (!msg) {
    return;
  }
  // dataset
  pplanner_interfaces__msg__ArucoData__Sequence__fini(&msg->dataset);
}

bool
pplanner_interfaces__msg__ArucoDataset__are_equal(const pplanner_interfaces__msg__ArucoDataset * lhs, const pplanner_interfaces__msg__ArucoDataset * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // dataset
  if (!pplanner_interfaces__msg__ArucoData__Sequence__are_equal(
      &(lhs->dataset), &(rhs->dataset)))
  {
    return false;
  }
  return true;
}

bool
pplanner_interfaces__msg__ArucoDataset__copy(
  const pplanner_interfaces__msg__ArucoDataset * input,
  pplanner_interfaces__msg__ArucoDataset * output)
{
  if (!input || !output) {
    return false;
  }
  // dataset
  if (!pplanner_interfaces__msg__ArucoData__Sequence__copy(
      &(input->dataset), &(output->dataset)))
  {
    return false;
  }
  return true;
}

pplanner_interfaces__msg__ArucoDataset *
pplanner_interfaces__msg__ArucoDataset__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pplanner_interfaces__msg__ArucoDataset * msg = (pplanner_interfaces__msg__ArucoDataset *)allocator.allocate(sizeof(pplanner_interfaces__msg__ArucoDataset), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pplanner_interfaces__msg__ArucoDataset));
  bool success = pplanner_interfaces__msg__ArucoDataset__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pplanner_interfaces__msg__ArucoDataset__destroy(pplanner_interfaces__msg__ArucoDataset * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pplanner_interfaces__msg__ArucoDataset__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pplanner_interfaces__msg__ArucoDataset__Sequence__init(pplanner_interfaces__msg__ArucoDataset__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pplanner_interfaces__msg__ArucoDataset * data = NULL;

  if (size) {
    data = (pplanner_interfaces__msg__ArucoDataset *)allocator.zero_allocate(size, sizeof(pplanner_interfaces__msg__ArucoDataset), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pplanner_interfaces__msg__ArucoDataset__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pplanner_interfaces__msg__ArucoDataset__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
pplanner_interfaces__msg__ArucoDataset__Sequence__fini(pplanner_interfaces__msg__ArucoDataset__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pplanner_interfaces__msg__ArucoDataset__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

pplanner_interfaces__msg__ArucoDataset__Sequence *
pplanner_interfaces__msg__ArucoDataset__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pplanner_interfaces__msg__ArucoDataset__Sequence * array = (pplanner_interfaces__msg__ArucoDataset__Sequence *)allocator.allocate(sizeof(pplanner_interfaces__msg__ArucoDataset__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pplanner_interfaces__msg__ArucoDataset__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pplanner_interfaces__msg__ArucoDataset__Sequence__destroy(pplanner_interfaces__msg__ArucoDataset__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pplanner_interfaces__msg__ArucoDataset__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pplanner_interfaces__msg__ArucoDataset__Sequence__are_equal(const pplanner_interfaces__msg__ArucoDataset__Sequence * lhs, const pplanner_interfaces__msg__ArucoDataset__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pplanner_interfaces__msg__ArucoDataset__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pplanner_interfaces__msg__ArucoDataset__Sequence__copy(
  const pplanner_interfaces__msg__ArucoDataset__Sequence * input,
  pplanner_interfaces__msg__ArucoDataset__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pplanner_interfaces__msg__ArucoDataset);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pplanner_interfaces__msg__ArucoDataset * data =
      (pplanner_interfaces__msg__ArucoDataset *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pplanner_interfaces__msg__ArucoDataset__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pplanner_interfaces__msg__ArucoDataset__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pplanner_interfaces__msg__ArucoDataset__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
