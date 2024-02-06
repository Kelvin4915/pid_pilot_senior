// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pplanner_interfaces:msg/PathGridset.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pplanner_interfaces/msg/detail/path_gridset__rosidl_typesupport_introspection_c.h"
#include "pplanner_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pplanner_interfaces/msg/detail/path_gridset__functions.h"
#include "pplanner_interfaces/msg/detail/path_gridset__struct.h"


// Include directives for member types
// Member `path`
#include "pplanner_interfaces/msg/path_grid.h"
// Member `path`
#include "pplanner_interfaces/msg/detail/path_grid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pplanner_interfaces__msg__PathGridset__init(message_memory);
}

void pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_fini_function(void * message_memory)
{
  pplanner_interfaces__msg__PathGridset__fini(message_memory);
}

size_t pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__size_function__PathGridset__path(
  const void * untyped_member)
{
  const pplanner_interfaces__msg__PathGrid__Sequence * member =
    (const pplanner_interfaces__msg__PathGrid__Sequence *)(untyped_member);
  return member->size;
}

const void * pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__get_const_function__PathGridset__path(
  const void * untyped_member, size_t index)
{
  const pplanner_interfaces__msg__PathGrid__Sequence * member =
    (const pplanner_interfaces__msg__PathGrid__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__get_function__PathGridset__path(
  void * untyped_member, size_t index)
{
  pplanner_interfaces__msg__PathGrid__Sequence * member =
    (pplanner_interfaces__msg__PathGrid__Sequence *)(untyped_member);
  return &member->data[index];
}

void pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__fetch_function__PathGridset__path(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pplanner_interfaces__msg__PathGrid * item =
    ((const pplanner_interfaces__msg__PathGrid *)
    pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__get_const_function__PathGridset__path(untyped_member, index));
  pplanner_interfaces__msg__PathGrid * value =
    (pplanner_interfaces__msg__PathGrid *)(untyped_value);
  *value = *item;
}

void pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__assign_function__PathGridset__path(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pplanner_interfaces__msg__PathGrid * item =
    ((pplanner_interfaces__msg__PathGrid *)
    pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__get_function__PathGridset__path(untyped_member, index));
  const pplanner_interfaces__msg__PathGrid * value =
    (const pplanner_interfaces__msg__PathGrid *)(untyped_value);
  *item = *value;
}

bool pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__resize_function__PathGridset__path(
  void * untyped_member, size_t size)
{
  pplanner_interfaces__msg__PathGrid__Sequence * member =
    (pplanner_interfaces__msg__PathGrid__Sequence *)(untyped_member);
  pplanner_interfaces__msg__PathGrid__Sequence__fini(member);
  return pplanner_interfaces__msg__PathGrid__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_message_member_array[1] = {
  {
    "path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pplanner_interfaces__msg__PathGridset, path),  // bytes offset in struct
    NULL,  // default value
    pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__size_function__PathGridset__path,  // size() function pointer
    pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__get_const_function__PathGridset__path,  // get_const(index) function pointer
    pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__get_function__PathGridset__path,  // get(index) function pointer
    pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__fetch_function__PathGridset__path,  // fetch(index, &value) function pointer
    pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__assign_function__PathGridset__path,  // assign(index, value) function pointer
    pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__resize_function__PathGridset__path  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_message_members = {
  "pplanner_interfaces__msg",  // message namespace
  "PathGridset",  // message name
  1,  // number of fields
  sizeof(pplanner_interfaces__msg__PathGridset),
  pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_message_member_array,  // message members
  pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_init_function,  // function to initialize message memory (memory has to be allocated)
  pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_message_type_support_handle = {
  0,
  &pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pplanner_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pplanner_interfaces, msg, PathGridset)() {
  pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pplanner_interfaces, msg, PathGrid)();
  if (!pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_message_type_support_handle.typesupport_identifier) {
    pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pplanner_interfaces__msg__PathGridset__rosidl_typesupport_introspection_c__PathGridset_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
