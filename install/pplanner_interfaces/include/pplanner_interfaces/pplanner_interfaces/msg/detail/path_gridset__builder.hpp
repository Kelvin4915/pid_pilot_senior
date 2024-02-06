// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pplanner_interfaces:msg/PathGridset.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__BUILDER_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pplanner_interfaces/msg/detail/path_gridset__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pplanner_interfaces
{

namespace msg
{

namespace builder
{

class Init_PathGridset_path
{
public:
  Init_PathGridset_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pplanner_interfaces::msg::PathGridset path(::pplanner_interfaces::msg::PathGridset::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pplanner_interfaces::msg::PathGridset msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pplanner_interfaces::msg::PathGridset>()
{
  return pplanner_interfaces::msg::builder::Init_PathGridset_path();
}

}  // namespace pplanner_interfaces

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRIDSET__BUILDER_HPP_
