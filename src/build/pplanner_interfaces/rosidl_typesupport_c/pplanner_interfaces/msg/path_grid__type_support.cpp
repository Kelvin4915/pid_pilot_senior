// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from pplanner_interfaces:msg/PathGrid.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "pplanner_interfaces/msg/detail/path_grid__struct.h"
#include "pplanner_interfaces/msg/detail/path_grid__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace pplanner_interfaces
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _PathGrid_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PathGrid_type_support_ids_t;

static const _PathGrid_type_support_ids_t _PathGrid_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PathGrid_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PathGrid_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PathGrid_type_support_symbol_names_t _PathGrid_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pplanner_interfaces, msg, PathGrid)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pplanner_interfaces, msg, PathGrid)),
  }
};

typedef struct _PathGrid_type_support_data_t
{
  void * data[2];
} _PathGrid_type_support_data_t;

static _PathGrid_type_support_data_t _PathGrid_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PathGrid_message_typesupport_map = {
  2,
  "pplanner_interfaces",
  &_PathGrid_message_typesupport_ids.typesupport_identifier[0],
  &_PathGrid_message_typesupport_symbol_names.symbol_name[0],
  &_PathGrid_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PathGrid_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PathGrid_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace pplanner_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, pplanner_interfaces, msg, PathGrid)() {
  return &::pplanner_interfaces::msg::rosidl_typesupport_c::PathGrid_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
