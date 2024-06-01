// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pplanner_interfaces:msg/PathGridset.idl
// generated code does not contain a copyright notice
#include "pplanner_interfaces/msg/detail/path_gridset__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `path`
#include "pplanner_interfaces/msg/detail/path_grid__functions.h"

bool
pplanner_interfaces__msg__PathGridset__init(pplanner_interfaces__msg__PathGridset * msg)
{
  if (!msg) {
    return false;
  }
  // path
  if (!pplanner_interfaces__msg__PathGrid__Sequence__init(&msg->path, 0)) {
    pplanner_interfaces__msg__PathGridset__fini(msg);
    return false;
  }
  return true;
}

void
pplanner_interfaces__msg__PathGridset__fini(pplanner_interfaces__msg__PathGridset * msg)
{
  if (!msg) {
    return;
  }
  // path
  pplanner_interfaces__msg__PathGrid__Sequence__fini(&msg->path);
}

bool
pplanner_interfaces__msg__PathGridset__are_equal(const pplanner_interfaces__msg__PathGridset * lhs, const pplanner_interfaces__msg__PathGridset * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // path
  if (!pplanner_interfaces__msg__PathGrid__Sequence__are_equal(
      &(lhs->path), &(rhs->path)))
  {
    return false;
  }
  return true;
}

bool
pplanner_interfaces__msg__PathGridset__copy(
  const pplanner_interfaces__msg__PathGridset * input,
  pplanner_interfaces__msg__PathGridset * output)
{
  if (!input || !output) {
    return false;
  }
  // path
  if (!pplanner_interfaces__msg__PathGrid__Sequence__copy(
      &(input->path), &(output->path)))
  {
    return false;
  }
  return true;
}

pplanner_interfaces__msg__PathGridset *
pplanner_interfaces__msg__PathGridset__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pplanner_interfaces__msg__PathGridset * msg = (pplanner_interfaces__msg__PathGridset *)allocator.allocate(sizeof(pplanner_interfaces__msg__PathGridset), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pplanner_interfaces__msg__PathGridset));
  bool success = pplanner_interfaces__msg__PathGridset__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pplanner_interfaces__msg__PathGridset__destroy(pplanner_interfaces__msg__PathGridset * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pplanner_interfaces__msg__PathGridset__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pplanner_interfaces__msg__PathGridset__Sequence__init(pplanner_interfaces__msg__PathGridset__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pplanner_interfaces__msg__PathGridset * data = NULL;

  if (size) {
    data = (pplanner_interfaces__msg__PathGridset *)allocator.zero_allocate(size, sizeof(pplanner_interfaces__msg__PathGridset), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pplanner_interfaces__msg__PathGridset__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pplanner_interfaces__msg__PathGridset__fini(&data[i - 1]);
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
pplanner_interfaces__msg__PathGridset__Sequence__fini(pplanner_interfaces__msg__PathGridset__Sequence * array)
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
      pplanner_interfaces__msg__PathGridset__fini(&array->data[i]);
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

pplanner_interfaces__msg__PathGridset__Sequence *
pplanner_interfaces__msg__PathGridset__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pplanner_interfaces__msg__PathGridset__Sequence * array = (pplanner_interfaces__msg__PathGridset__Sequence *)allocator.allocate(sizeof(pplanner_interfaces__msg__PathGridset__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pplanner_interfaces__msg__PathGridset__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pplanner_interfaces__msg__PathGridset__Sequence__destroy(pplanner_interfaces__msg__PathGridset__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pplanner_interfaces__msg__PathGridset__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pplanner_interfaces__msg__PathGridset__Sequence__are_equal(const pplanner_interfaces__msg__PathGridset__Sequence * lhs, const pplanner_interfaces__msg__PathGridset__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pplanner_interfaces__msg__PathGridset__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pplanner_interfaces__msg__PathGridset__Sequence__copy(
  const pplanner_interfaces__msg__PathGridset__Sequence * input,
  pplanner_interfaces__msg__PathGridset__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pplanner_interfaces__msg__PathGridset);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pplanner_interfaces__msg__PathGridset * data =
      (pplanner_interfaces__msg__PathGridset *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pplanner_interfaces__msg__PathGridset__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pplanner_interfaces__msg__PathGridset__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pplanner_interfaces__msg__PathGridset__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
