// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pplanner_interfaces:msg/ArucoData.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__TRAITS_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pplanner_interfaces/msg/detail/aruco_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pplanner_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArucoData & msg,
  std::ostream & out)
{
  out << "{";
  // member: id_data
  {
    out << "id_data: ";
    rosidl_generator_traits::value_to_yaml(msg.id_data, out);
    out << ", ";
  }

  // member: x_data
  {
    out << "x_data: ";
    rosidl_generator_traits::value_to_yaml(msg.x_data, out);
    out << ", ";
  }

  // member: y_data
  {
    out << "y_data: ";
    rosidl_generator_traits::value_to_yaml(msg.y_data, out);
    out << ", ";
  }

  // member: orientation_data
  {
    out << "orientation_data: ";
    rosidl_generator_traits::value_to_yaml(msg.orientation_data, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArucoData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id_data: ";
    rosidl_generator_traits::value_to_yaml(msg.id_data, out);
    out << "\n";
  }

  // member: x_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_data: ";
    rosidl_generator_traits::value_to_yaml(msg.x_data, out);
    out << "\n";
  }

  // member: y_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_data: ";
    rosidl_generator_traits::value_to_yaml(msg.y_data, out);
    out << "\n";
  }

  // member: orientation_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation_data: ";
    rosidl_generator_traits::value_to_yaml(msg.orientation_data, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArucoData & msg, bool use_flow_style = false)
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
  const pplanner_interfaces::msg::ArucoData & msg,
  std::ostream & out, size_t indentation = 0)
{
  pplanner_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pplanner_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const pplanner_interfaces::msg::ArucoData & msg)
{
  return pplanner_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pplanner_interfaces::msg::ArucoData>()
{
  return "pplanner_interfaces::msg::ArucoData";
}

template<>
inline const char * name<pplanner_interfaces::msg::ArucoData>()
{
  return "pplanner_interfaces/msg/ArucoData";
}

template<>
struct has_fixed_size<pplanner_interfaces::msg::ArucoData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pplanner_interfaces::msg::ArucoData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pplanner_interfaces::msg::ArucoData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__TRAITS_HPP_
