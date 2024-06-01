// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pplanner_interfaces:msg/PathGrid.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__TRAITS_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pplanner_interfaces/msg/detail/path_grid__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pplanner_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const PathGrid & msg,
  std::ostream & out)
{
  out << "{";
  // member: row
  {
    out << "row: ";
    rosidl_generator_traits::value_to_yaml(msg.row, out);
    out << ", ";
  }

  // member: col
  {
    out << "col: ";
    rosidl_generator_traits::value_to_yaml(msg.col, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PathGrid & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: row
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "row: ";
    rosidl_generator_traits::value_to_yaml(msg.row, out);
    out << "\n";
  }

  // member: col
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "col: ";
    rosidl_generator_traits::value_to_yaml(msg.col, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PathGrid & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace pplanner_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use pplanner_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pplanner_interfaces::msg::PathGrid & msg,
  std::ostream & out, size_t indentation = 0)
{
  pplanner_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pplanner_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const pplanner_interfaces::msg::PathGrid & msg)
{
  return pplanner_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pplanner_interfaces::msg::PathGrid>()
{
  return "pplanner_interfaces::msg::PathGrid";
}

template<>
inline const char * name<pplanner_interfaces::msg::PathGrid>()
{
  return "pplanner_interfaces/msg/PathGrid";
}

template<>
struct has_fixed_size<pplanner_interfaces::msg::PathGrid>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pplanner_interfaces::msg::PathGrid>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pplanner_interfaces::msg::PathGrid>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__TRAITS_HPP_
