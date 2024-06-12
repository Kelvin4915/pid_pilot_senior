// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pplanner_interfaces:msg/PathGridset.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__TRAITS_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pplanner_interfaces/msg/detail/path_gridset__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'path'
#include "pplanner_interfaces/msg/detail/path_grid__traits.hpp"

namespace pplanner_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const PathGridset & msg,
  std::ostream & out)
{
  out << "{";
  // member: path
  {
    if (msg.path.size() == 0) {
      out << "path: []";
    } else {
      out << "path: [";
      size_t pending_items = msg.path.size();
      for (auto item : msg.path) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PathGridset & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.path.size() == 0) {
      out << "path: []\n";
    } else {
      out << "path:\n";
      for (auto item : msg.path) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PathGridset & msg, bool use_flow_style = false)
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
  const pplanner_interfaces::msg::PathGridset & msg,
  std::ostream & out, size_t indentation = 0)
{
  pplanner_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pplanner_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const pplanner_interfaces::msg::PathGridset & msg)
{
  return pplanner_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pplanner_interfaces::msg::PathGridset>()
{
  return "pplanner_interfaces::msg::PathGridset";
}

template<>
inline const char * name<pplanner_interfaces::msg::PathGridset>()
{
  return "pplanner_interfaces/msg/PathGridset";
}

template<>
struct has_fixed_size<pplanner_interfaces::msg::PathGridset>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pplanner_interfaces::msg::PathGridset>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pplanner_interfaces::msg::PathGridset>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__TRAITS_HPP_
