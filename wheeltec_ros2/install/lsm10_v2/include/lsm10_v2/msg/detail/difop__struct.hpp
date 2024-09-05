// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lsm10_v2:msg/Difop.idl
// generated code does not contain a copyright notice

#ifndef LSM10_V2__MSG__DETAIL__DIFOP__STRUCT_HPP_
#define LSM10_V2__MSG__DETAIL__DIFOP__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__lsm10_v2__msg__Difop __attribute__((deprecated))
#else
# define DEPRECATED__lsm10_v2__msg__Difop __declspec(deprecated)
#endif

namespace lsm10_v2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Difop_
{
  using Type = Difop_<ContainerAllocator>;

  explicit Difop_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motorspeed = 0l;
    }
  }

  explicit Difop_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motorspeed = 0l;
    }
  }

  // field types and members
  using _motorspeed_type =
    int32_t;
  _motorspeed_type motorspeed;

  // setters for named parameter idiom
  Type & set__motorspeed(
    const int32_t & _arg)
  {
    this->motorspeed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lsm10_v2::msg::Difop_<ContainerAllocator> *;
  using ConstRawPtr =
    const lsm10_v2::msg::Difop_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lsm10_v2::msg::Difop_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lsm10_v2::msg::Difop_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lsm10_v2::msg::Difop_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lsm10_v2::msg::Difop_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lsm10_v2::msg::Difop_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lsm10_v2::msg::Difop_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lsm10_v2::msg::Difop_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lsm10_v2::msg::Difop_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lsm10_v2__msg__Difop
    std::shared_ptr<lsm10_v2::msg::Difop_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lsm10_v2__msg__Difop
    std::shared_ptr<lsm10_v2::msg::Difop_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Difop_ & other) const
  {
    if (this->motorspeed != other.motorspeed) {
      return false;
    }
    return true;
  }
  bool operator!=(const Difop_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Difop_

// alias to use template instance with default allocator
using Difop =
  lsm10_v2::msg::Difop_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lsm10_v2

#endif  // LSM10_V2__MSG__DETAIL__DIFOP__STRUCT_HPP_
