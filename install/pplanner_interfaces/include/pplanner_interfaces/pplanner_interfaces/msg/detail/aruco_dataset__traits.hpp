// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pplanner_interfaces:msg/ArucoDataset.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__TRAITS_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pplanner_interfaces/msg/detail/aruco_dataset__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'dataset'
#include "pplanner_interfaces/msg/detail/aruco_data__traits.hpp"

namespace pplanner_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArucoDataset & msg,
  std::ostream & out)
{
  out << "{";
  // member: dataset
  {
    if (msg.dataset.size() == 0) {
      out << "dataset: []";
    } else {
      out << "dataset: [";
      size_t pending_items = msg.dataset.size();
      for (auto item : msg.dataset) {
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
  const ArucoDataset & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dataset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.dataset.size() == 0) {
      out << "dataset: []\n";
    } else {
      out << "dataset:\n";
      for (auto item : msg.dataset) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArucoDataset & msg, bool use_flow_style = false)
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
  const pplanner_interfaces::msg::ArucoDataset & msg,
  std::ostream & out, size_t indentation = 0)
{
  pplanner_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pplanner_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const pplanner_interfaces::msg::ArucoDataset & msg)
{
  return pplanner_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pplanner_interfaces::msg::ArucoDataset>()
{
  return "pplanner_interfaces::msg::ArucoDataset";
}

template<>
inline const char * name<pplanner_interfaces::msg::ArucoDataset>()
{
  return "pplanner_interfaces/msg/ArucoDataset";
}

template<>
struct has_fixed_size<pplanner_interfaces::msg::ArucoDataset>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pplanner_interfaces::msg::ArucoDataset>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pplanner_interfaces::msg::ArucoDataset>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__TRAITS_HPP_
