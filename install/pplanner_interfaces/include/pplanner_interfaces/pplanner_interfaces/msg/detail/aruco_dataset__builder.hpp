// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pplanner_interfaces:msg/ArucoDataset.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__BUILDER_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pplanner_interfaces/msg/detail/aruco_dataset__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pplanner_interfaces
{

namespace msg
{

namespace builder
{

class Init_ArucoDataset_dataset
{
public:
  Init_ArucoDataset_dataset()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pplanner_interfaces::msg::ArucoDataset dataset(::pplanner_interfaces::msg::ArucoDataset::_dataset_type arg)
  {
    msg_.dataset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pplanner_interfaces::msg::ArucoDataset msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pplanner_interfaces::msg::ArucoDataset>()
{
  return pplanner_interfaces::msg::builder::Init_ArucoDataset_dataset();
}

}  // namespace pplanner_interfaces

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__BUILDER_HPP_
