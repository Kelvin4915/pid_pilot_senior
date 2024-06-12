// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pplanner_interfaces:msg/ArucoData.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__STRUCT_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pplanner_interfaces__msg__ArucoData __attribute__((deprecated))
#else
# define DEPRECATED__pplanner_interfaces__msg__ArucoData __declspec(deprecated)
#endif

namespace pplanner_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArucoData_
{
  using Type = ArucoData_<ContainerAllocator>;

  explicit ArucoData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id_data = 0ll;
      this->x_data = 0ll;
      this->y_data = 0ll;
      this->orientation_data = 0.0;
      this->status = "";
    }
  }

  explicit ArucoData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id_data = 0ll;
      this->x_data = 0ll;
      this->y_data = 0ll;
      this->orientation_data = 0.0;
      this->status = "";
    }
  }

  // field types and members
  using _id_data_type =
    int64_t;
  _id_data_type id_data;
  using _x_data_type =
    int64_t;
  _x_data_type x_data;
  using _y_data_type =
    int64_t;
  _y_data_type y_data;
  using _orientation_data_type =
    double;
  _orientation_data_type orientation_data;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__id_data(
    const int64_t & _arg)
  {
    this->id_data = _arg;
    return *this;
  }
  Type & set__x_data(
    const int64_t & _arg)
  {
    this->x_data = _arg;
    return *this;
  }
  Type & set__y_data(
    const int64_t & _arg)
  {
    this->y_data = _arg;
    return *this;
  }
  Type & set__orientation_data(
    const double & _arg)
  {
    this->orientation_data = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pplanner_interfaces::msg::ArucoData_<ContainerAllocator> *;
  using ConstRawPtr =
    const pplanner_interfaces::msg::ArucoData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pplanner_interfaces::msg::ArucoData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pplanner_interfaces::msg::ArucoData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pplanner_interfaces::msg::ArucoData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pplanner_interfaces::msg::ArucoData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pplanner_interfaces::msg::ArucoData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pplanner_interfaces::msg::ArucoData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pplanner_interfaces::msg::ArucoData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pplanner_interfaces::msg::ArucoData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pplanner_interfaces__msg__ArucoData
    std::shared_ptr<pplanner_interfaces::msg::ArucoData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pplanner_interfaces__msg__ArucoData
    std::shared_ptr<pplanner_interfaces::msg::ArucoData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArucoData_ & other) const
  {
    if (this->id_data != other.id_data) {
      return false;
    }
    if (this->x_data != other.x_data) {
      return false;
    }
    if (this->y_data != other.y_data) {
      return false;
    }
    if (this->orientation_data != other.orientation_data) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArucoData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArucoData_

// alias to use template instance with default allocator
using ArucoData =
  pplanner_interfaces::msg::ArucoData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pplanner_interfaces

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATA__STRUCT_HPP_
