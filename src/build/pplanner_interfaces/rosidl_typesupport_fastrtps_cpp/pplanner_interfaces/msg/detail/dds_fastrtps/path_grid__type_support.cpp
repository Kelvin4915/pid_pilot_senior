// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from pplanner_interfaces:msg/PathGrid.idl
// generated code does not contain a copyright notice
#include "pplanner_interfaces/msg/detail/path_grid__rosidl_typesupport_fastrtps_cpp.hpp"
#include "pplanner_interfaces/msg/detail/path_grid__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace pplanner_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pplanner_interfaces
cdr_serialize(
  const pplanner_interfaces::msg::PathGrid & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: row
  cdr << ros_message.row;
  // Member: col
  cdr << ros_message.col;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pplanner_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  pplanner_interfaces::msg::PathGrid & ros_message)
{
  // Member: row
  cdr >> ros_message.row;

  // Member: col
  cdr >> ros_message.col;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pplanner_interfaces
get_serialized_size(
  const pplanner_interfaces::msg::PathGrid & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: row
  {
    size_t item_size = sizeof(ros_message.row);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: col
  {
    size_t item_size = sizeof(ros_message.col);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_pplanner_interfaces
max_serialized_size_PathGrid(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: row
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: col
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = pplanner_interfaces::msg::PathGrid;
    is_plain =
      (
      offsetof(DataType, col) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PathGrid__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const pplanner_interfaces::msg::PathGrid *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PathGrid__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<pplanner_interfaces::msg::PathGrid *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PathGrid__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const pplanner_interfaces::msg::PathGrid *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PathGrid__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PathGrid(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PathGrid__callbacks = {
  "pplanner_interfaces::msg",
  "PathGrid",
  _PathGrid__cdr_serialize,
  _PathGrid__cdr_deserialize,
  _PathGrid__get_serialized_size,
  _PathGrid__max_serialized_size
};

static rosidl_message_type_support_t _PathGrid__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PathGrid__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace pplanner_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_pplanner_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<pplanner_interfaces::msg::PathGrid>()
{
  return &pplanner_interfaces::msg::typesupport_fastrtps_cpp::_PathGrid__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, pplanner_interfaces, msg, PathGrid)() {
  return &pplanner_interfaces::msg::typesupport_fastrtps_cpp::_PathGrid__handle;
}

#ifdef __cplusplus
}
#endif
