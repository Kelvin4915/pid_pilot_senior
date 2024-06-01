// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pplanner_interfaces:msg/PathGrid.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__BUILDER_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pplanner_interfaces/msg/detail/path_grid__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pplanner_interfaces
{

namespace msg
{

namespace builder
{

class Init_PathGrid_col
{
public:
  explicit Init_PathGrid_col(::pplanner_interfaces::msg::PathGrid & msg)
  : msg_(msg)
  {}
  ::pplanner_interfaces::msg::PathGrid col(::pplanner_interfaces::msg::PathGrid::_col_type arg)
  {
    msg_.col = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pplanner_interfaces::msg::PathGrid msg_;
};

class Init_PathGrid_row
{
public:
  Init_PathGrid_row()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathGrid_col row(::pplanner_interfaces::msg::PathGrid::_row_type arg)
  {
    msg_.row = std::move(arg);
    return Init_PathGrid_col(msg_);
  }

private:
  ::pplanner_interfaces::msg::PathGrid msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pplanner_interfaces::msg::PathGrid>()
{
  return pplanner_interfaces::msg::builder::Init_PathGrid_row();
}

}  // namespace pplanner_interfaces

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__BUILDER_HPP_
