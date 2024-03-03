// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pplanner_interfaces:msg/ArucoDataset.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pplanner_interfaces/msg/detail/aruco_dataset__rosidl_typesupport_introspection_c.h"
#include "pplanner_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pplanner_interfaces/msg/detail/aruco_dataset__functions.h"
#include "pplanner_interfaces/msg/detail/aruco_dataset__struct.h"


// Include directives for member types
// Member `dataset`
#include "pplanner_interfaces/msg/aruco_data.h"
// Member `dataset`
#include "pplanner_interfaces/msg/detail/aruco_data__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pplanner_interfaces__msg__ArucoDataset__init(message_memory);
}

void pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_fini_function(void * message_memory)
{
  pplanner_interfaces__msg__ArucoDataset__fini(message_memory);
}

size_t pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__size_function__ArucoDataset__dataset(
  const void * untyped_member)
{
  const pplanner_interfaces__msg__ArucoData__Sequence * member =
    (const pplanner_interfaces__msg__ArucoData__Sequence *)(untyped_member);
  return member->size;
}

const void * pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__get_const_function__ArucoDataset__dataset(
  const void * untyped_member, size_t index)
{
  const pplanner_interfaces__msg__ArucoData__Sequence * member =
    (const pplanner_interfaces__msg__ArucoData__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__get_function__ArucoDataset__dataset(
  void * untyped_member, size_t index)
{
  pplanner_interfaces__msg__ArucoData__Sequence * member =
    (pplanner_interfaces__msg__ArucoData__Sequence *)(untyped_member);
  return &member->data[index];
}

void pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__fetch_function__ArucoDataset__dataset(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pplanner_interfaces__msg__ArucoData * item =
    ((const pplanner_interfaces__msg__ArucoData *)
    pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__get_const_function__ArucoDataset__dataset(untyped_member, index));
  pplanner_interfaces__msg__ArucoData * value =
    (pplanner_interfaces__msg__ArucoData *)(untyped_value);
  *value = *item;
}

void pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__assign_function__ArucoDataset__dataset(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pplanner_interfaces__msg__ArucoData * item =
    ((pplanner_interfaces__msg__ArucoData *)
    pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__get_function__ArucoDataset__dataset(untyped_member, index));
  const pplanner_interfaces__msg__ArucoData * value =
    (const pplanner_interfaces__msg__ArucoData *)(untyped_value);
  *item = *value;
}

bool pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__resize_function__ArucoDataset__dataset(
  void * untyped_member, size_t size)
{
  pplanner_interfaces__msg__ArucoData__Sequence * member =
    (pplanner_interfaces__msg__ArucoData__Sequence *)(untyped_member);
  pplanner_interfaces__msg__ArucoData__Sequence__fini(member);
  return pplanner_interfaces__msg__ArucoData__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_message_member_array[1] = {
  {
    "dataset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pplanner_interfaces__msg__ArucoDataset, dataset),  // bytes offset in struct
    NULL,  // default value
    pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__size_function__ArucoDataset__dataset,  // size() function pointer
    pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__get_const_function__ArucoDataset__dataset,  // get_const(index) function pointer
    pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__get_function__ArucoDataset__dataset,  // get(index) function pointer
    pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__fetch_function__ArucoDataset__dataset,  // fetch(index, &value) function pointer
    pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__assign_function__ArucoDataset__dataset,  // assign(index, value) function pointer
    pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__resize_function__ArucoDataset__dataset  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_message_members = {
  "pplanner_interfaces__msg",  // message namespace
  "ArucoDataset",  // message name
  1,  // number of fields
  sizeof(pplanner_interfaces__msg__ArucoDataset),
  pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_message_member_array,  // message members
  pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_init_function,  // function to initialize message memory (memory has to be allocated)
  pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_message_type_support_handle = {
  0,
  &pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pplanner_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pplanner_interfaces, msg, ArucoDataset)() {
  pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pplanner_interfaces, msg, ArucoData)();
  if (!pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_message_type_support_handle.typesupport_identifier) {
    pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pplanner_interfaces__msg__ArucoDataset__rosidl_typesupport_introspection_c__ArucoDataset_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
