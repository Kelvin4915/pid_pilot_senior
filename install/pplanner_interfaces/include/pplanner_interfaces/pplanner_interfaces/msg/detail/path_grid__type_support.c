// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pplanner_interfaces:msg/PathGrid.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pplanner_interfaces/msg/detail/path_grid__rosidl_typesupport_introspection_c.h"
#include "pplanner_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pplanner_interfaces/msg/detail/path_grid__functions.h"
#include "pplanner_interfaces/msg/detail/path_grid__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pplanner_interfaces__msg__PathGrid__init(message_memory);
}

void pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_fini_function(void * message_memory)
{
  pplanner_interfaces__msg__PathGrid__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_message_member_array[2] = {
  {
    "row",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pplanner_interfaces__msg__PathGrid, row),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "col",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pplanner_interfaces__msg__PathGrid, col),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_message_members = {
  "pplanner_interfaces__msg",  // message namespace
  "PathGrid",  // message name
  2,  // number of fields
  sizeof(pplanner_interfaces__msg__PathGrid),
  pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_message_member_array,  // message members
  pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_init_function,  // function to initialize message memory (memory has to be allocated)
  pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_message_type_support_handle = {
  0,
  &pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pplanner_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pplanner_interfaces, msg, PathGrid)() {
  if (!pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_message_type_support_handle.typesupport_identifier) {
    pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pplanner_interfaces__msg__PathGrid__rosidl_typesupport_introspection_c__PathGrid_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
