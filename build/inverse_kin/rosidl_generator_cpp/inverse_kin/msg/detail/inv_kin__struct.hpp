// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inverse_kin:msg/InvKin.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__MSG__DETAIL__INV_KIN__STRUCT_HPP_
#define INVERSE_KIN__MSG__DETAIL__INV_KIN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'scara_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'ee_pos'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'ee_angl'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inverse_kin__msg__InvKin __attribute__((deprecated))
#else
# define DEPRECATED__inverse_kin__msg__InvKin __declspec(deprecated)
#endif

namespace inverse_kin
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InvKin_
{
  using Type = InvKin_<ContainerAllocator>;

  explicit InvKin_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : scara_pose(_init),
    ee_pos(_init),
    ee_angl(_init)
  {
    (void)_init;
  }

  explicit InvKin_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : scara_pose(_alloc, _init),
    ee_pos(_alloc, _init),
    ee_angl(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _scara_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _scara_pose_type scara_pose;
  using _ee_pos_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _ee_pos_type ee_pos;
  using _ee_angl_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _ee_angl_type ee_angl;

  // setters for named parameter idiom
  Type & set__scara_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->scara_pose = _arg;
    return *this;
  }
  Type & set__ee_pos(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->ee_pos = _arg;
    return *this;
  }
  Type & set__ee_angl(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->ee_angl = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inverse_kin::msg::InvKin_<ContainerAllocator> *;
  using ConstRawPtr =
    const inverse_kin::msg::InvKin_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inverse_kin::msg::InvKin_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inverse_kin::msg::InvKin_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inverse_kin::msg::InvKin_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inverse_kin::msg::InvKin_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inverse_kin::msg::InvKin_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inverse_kin::msg::InvKin_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inverse_kin::msg::InvKin_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inverse_kin::msg::InvKin_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inverse_kin__msg__InvKin
    std::shared_ptr<inverse_kin::msg::InvKin_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inverse_kin__msg__InvKin
    std::shared_ptr<inverse_kin::msg::InvKin_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InvKin_ & other) const
  {
    if (this->scara_pose != other.scara_pose) {
      return false;
    }
    if (this->ee_pos != other.ee_pos) {
      return false;
    }
    if (this->ee_angl != other.ee_angl) {
      return false;
    }
    return true;
  }
  bool operator!=(const InvKin_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InvKin_

// alias to use template instance with default allocator
using InvKin =
  inverse_kin::msg::InvKin_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inverse_kin

#endif  // INVERSE_KIN__MSG__DETAIL__INV_KIN__STRUCT_HPP_
