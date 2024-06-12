// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from pplanner_interfaces:msg/PathGridset.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "pplanner_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "pplanner_interfaces/msg/detail/path_gridset__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace pplanner_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pplanner_interfaces
cdr_serialize(
  const pplanner_interfaces::msg::PathGridset & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pplanner_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  pplanner_interfaces::msg::PathGridset & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pplanner_interfaces
get_serialized_size(
  const pplanner_interfaces::msg::PathGridset & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pplanner_interfaces
max_serialized_size_PathGridset(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace pplanner_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pplanner_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pplanner_interfaces, msg, PathGridset)();

#ifdef __cplusplus
}
#endif

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
