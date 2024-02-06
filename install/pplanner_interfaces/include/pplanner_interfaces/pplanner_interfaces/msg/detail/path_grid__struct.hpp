// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pplanner_interfaces:msg/PathGrid.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__STRUCT_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pplanner_interfaces__msg__PathGrid __attribute__((deprecated))
#else
# define DEPRECATED__pplanner_interfaces__msg__PathGrid __declspec(deprecated)
#endif

namespace pplanner_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PathGrid_
{
  using Type = PathGrid_<ContainerAllocator>;

  explicit PathGrid_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->row = 0ll;
      this->col = 0ll;
    }
  }

  explicit PathGrid_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->row = 0ll;
      this->col = 0ll;
    }
  }

  // field types and members
  using _row_type =
    int64_t;
  _row_type row;
  using _col_type =
    int64_t;
  _col_type col;

  // setters for named parameter idiom
  Type & set__row(
    const int64_t & _arg)
  {
    this->row = _arg;
    return *this;
  }
  Type & set__col(
    const int64_t & _arg)
  {
    this->col = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pplanner_interfaces::msg::PathGrid_<ContainerAllocator> *;
  using ConstRawPtr =
    const pplanner_interfaces::msg::PathGrid_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pplanner_interfaces::msg::PathGrid_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pplanner_interfaces::msg::PathGrid_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pplanner_interfaces::msg::PathGrid_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pplanner_interfaces::msg::PathGrid_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pplanner_interfaces::msg::PathGrid_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pplanner_interfaces::msg::PathGrid_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pplanner_interfaces::msg::PathGrid_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pplanner_interfaces::msg::PathGrid_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pplanner_interfaces__msg__PathGrid
    std::shared_ptr<pplanner_interfaces::msg::PathGrid_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pplanner_interfaces__msg__PathGrid
    std::shared_ptr<pplanner_interfaces::msg::PathGrid_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PathGrid_ & other) const
  {
    if (this->row != other.row) {
      return false;
    }
    if (this->col != other.col) {
      return false;
    }
    return true;
  }
  bool operator!=(const PathGrid_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PathGrid_

// alias to use template instance with default allocator
using PathGrid =
  pplanner_interfaces::msg::PathGrid_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pplanner_interfaces

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__PATH_GRID__STRUCT_HPP_
