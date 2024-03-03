// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pplanner_interfaces:msg/ArucoDataset.idl
// generated code does not contain a copyright notice

#ifndef PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__STRUCT_HPP_
#define PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'dataset'
#include "pplanner_interfaces/msg/detail/aruco_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pplanner_interfaces__msg__ArucoDataset __attribute__((deprecated))
#else
# define DEPRECATED__pplanner_interfaces__msg__ArucoDataset __declspec(deprecated)
#endif

namespace pplanner_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArucoDataset_
{
  using Type = ArucoDataset_<ContainerAllocator>;

  explicit ArucoDataset_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ArucoDataset_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _dataset_type =
    std::vector<pplanner_interfaces::msg::ArucoData_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pplanner_interfaces::msg::ArucoData_<ContainerAllocator>>>;
  _dataset_type dataset;

  // setters for named parameter idiom
  Type & set__dataset(
    const std::vector<pplanner_interfaces::msg::ArucoData_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pplanner_interfaces::msg::ArucoData_<ContainerAllocator>>> & _arg)
  {
    this->dataset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator> *;
  using ConstRawPtr =
    const pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pplanner_interfaces__msg__ArucoDataset
    std::shared_ptr<pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pplanner_interfaces__msg__ArucoDataset
    std::shared_ptr<pplanner_interfaces::msg::ArucoDataset_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArucoDataset_ & other) const
  {
    if (this->dataset != other.dataset) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArucoDataset_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArucoDataset_

// alias to use template instance with default allocator
using ArucoDataset =
  pplanner_interfaces::msg::ArucoDataset_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pplanner_interfaces

#endif  // PPLANNER_INTERFACES__MSG__DETAIL__ARUCO_DATASET__STRUCT_HPP_
