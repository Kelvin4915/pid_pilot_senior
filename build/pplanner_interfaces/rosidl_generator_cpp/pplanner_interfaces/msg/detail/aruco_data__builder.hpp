// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pplanner_interfaces:msg/ArucoData.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__BUILDER_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pplanner_interfaces/msg/detail/aruco_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pplanner_interfaces
{

namespace msg
{

namespace builder
{

class Init_ArucoData_status
{
public:
  explicit Init_ArucoData_status(::pplanner_interfaces::msg::ArucoData & msg)
  : msg_(msg)
  {}
  ::pplanner_interfaces::msg::ArucoData status(::pplanner_interfaces::msg::ArucoData::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pplanner_interfaces::msg::ArucoData msg_;
};

class Init_ArucoData_orientation_data
{
public:
  explicit Init_ArucoData_orientation_data(::pplanner_interfaces::msg::ArucoData & msg)
  : msg_(msg)
  {}
  Init_ArucoData_status orientation_data(::pplanner_interfaces::msg::ArucoData::_orientation_data_type arg)
  {
    msg_.orientation_data = std::move(arg);
    return Init_ArucoData_status(msg_);
  }

private:
  ::pplanner_interfaces::msg::ArucoData msg_;
};

class Init_ArucoData_y_data
{
public:
  explicit Init_ArucoData_y_data(::pplanner_interfaces::msg::ArucoData & msg)
  : msg_(msg)
  {}
  Init_ArucoData_orientation_data y_data(::pplanner_interfaces::msg::ArucoData::_y_data_type arg)
  {
    msg_.y_data = std::move(arg);
    return Init_ArucoData_orientation_data(msg_);
  }

private:
  ::pplanner_interfaces::msg::ArucoData msg_;
};

class Init_ArucoData_x_data
{
public:
  explicit Init_ArucoData_x_data(::pplanner_interfaces::msg::ArucoData & msg)
  : msg_(msg)
  {}
  Init_ArucoData_y_data x_data(::pplanner_interfaces::msg::ArucoData::_x_data_type arg)
  {
    msg_.x_data = std::move(arg);
    return Init_ArucoData_y_data(msg_);
  }

private:
  ::pplanner_interfaces::msg::ArucoData msg_;
};

class Init_ArucoData_id_data
{
public:
  Init_ArucoData_id_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArucoData_x_data id_data(::pplanner_interfaces::msg::ArucoData::_id_data_type arg)
  {
    msg_.id_data = std::move(arg);
    return Init_ArucoData_x_data(msg_);
  }

private:
  ::pplanner_interfaces::msg::ArucoData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pplanner_interfaces::msg::ArucoData>()
{
  return pplanner_interfaces::msg::builder::Init_ArucoData_id_data();
}

}  // namespace pplanner_interfaces

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__BUILDER_HPP_
