// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pplanner_interfaces:msg/ArucoData.idl
// generated code does not contain a copyright notice
#include "pplanner_interfaces/msg/detail/aruco_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `status`
#include "rosidl_runtime_c/string_functions.h"

bool
pplanner_interfaces__msg__ArucoData__init(pplanner_interfaces__msg__ArucoData * msg)
{
  if (!msg) {
    return false;
  }
  // id_data
  // x_data
  // y_data
  // orientation_data
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    pplanner_interfaces__msg__ArucoData__fini(msg);
    return false;
  }
  return true;
}

void
pplanner_interfaces__msg__ArucoData__fini(pplanner_interfaces__msg__ArucoData * msg)
{
  if (!msg) {
    return;
  }
  // id_data
  // x_data
  // y_data
  // orientation_data
  // status
  rosidl_runtime_c__String__fini(&msg->status);
}

bool
pplanner_interfaces__msg__ArucoData__are_equal(const pplanner_interfaces__msg__ArucoData * lhs, const pplanner_interfaces__msg__ArucoData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id_data
  if (lhs->id_data != rhs->id_data) {
    return false;
  }
  // x_data
  if (lhs->x_data != rhs->x_data) {
    return false;
  }
  // y_data
  if (lhs->y_data != rhs->y_data) {
    return false;
  }
  // orientation_data
  if (lhs->orientation_data != rhs->orientation_data) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  return true;
}

bool
pplanner_interfaces__msg__ArucoData__copy(
  const pplanner_interfaces__msg__ArucoData * input,
  pplanner_interfaces__msg__ArucoData * output)
{
  if (!input || !output) {
    return false;
  }
  // id_data
  output->id_data = input->id_data;
  // x_data
  output->x_data = input->x_data;
  // y_data
  output->y_data = input->y_data;
  // orientation_data
  output->orientation_data = input->orientation_data;
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  return true;
}

pplanner_interfaces__msg__ArucoData *
pplanner_interfaces__msg__ArucoData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pplanner_interfaces__msg__ArucoData * msg = (pplanner_interfaces__msg__ArucoData *)allocator.allocate(sizeof(pplanner_interfaces__msg__ArucoData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pplanner_interfaces__msg__ArucoData));
  bool success = pplanner_interfaces__msg__ArucoData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pplanner_interfaces__msg__ArucoData__destroy(pplanner_interfaces__msg__ArucoData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pplanner_interfaces__msg__ArucoData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pplanner_interfaces__msg__ArucoData__Sequence__init(pplanner_interfaces__msg__ArucoData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pplanner_interfaces__msg__ArucoData * data = NULL;

  if (size) {
    data = (pplanner_interfaces__msg__ArucoData *)allocator.zero_allocate(size, sizeof(pplanner_interfaces__msg__ArucoData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pplanner_interfaces__msg__ArucoData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pplanner_interfaces__msg__ArucoData__fini(&data[i - 1]);
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
pplanner_interfaces__msg__ArucoData__Sequence__fini(pplanner_interfaces__msg__ArucoData__Sequence * array)
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
      pplanner_interfaces__msg__ArucoData__fini(&array->data[i]);
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

pplanner_interfaces__msg__ArucoData__Sequence *
pplanner_interfaces__msg__ArucoData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pplanner_interfaces__msg__ArucoData__Sequence * array = (pplanner_interfaces__msg__ArucoData__Sequence *)allocator.allocate(sizeof(pplanner_interfaces__msg__ArucoData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pplanner_interfaces__msg__ArucoData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pplanner_interfaces__msg__ArucoData__Sequence__destroy(pplanner_interfaces__msg__ArucoData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pplanner_interfaces__msg__ArucoData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pplanner_interfaces__msg__ArucoData__Sequence__are_equal(const pplanner_interfaces__msg__ArucoData__Sequence * lhs, const pplanner_interfaces__msg__ArucoData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pplanner_interfaces__msg__ArucoData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pplanner_interfaces__msg__ArucoData__Sequence__copy(
  const pplanner_interfaces__msg__ArucoData__Sequence * input,
  pplanner_interfaces__msg__ArucoData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pplanner_interfaces__msg__ArucoData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pplanner_interfaces__msg__ArucoData * data =
      (pplanner_interfaces__msg__ArucoData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pplanner_interfaces__msg__ArucoData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pplanner_interfaces__msg__ArucoData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pplanner_interfaces__msg__ArucoData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
