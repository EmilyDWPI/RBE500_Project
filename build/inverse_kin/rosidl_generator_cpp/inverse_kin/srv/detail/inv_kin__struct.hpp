// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inverse_kin:srv/InvKin.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__SRV__DETAIL__INV_KIN__STRUCT_HPP_
#define INVERSE_KIN__SRV__DETAIL__INV_KIN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__inverse_kin__srv__InvKin_Request __attribute__((deprecated))
#else
# define DEPRECATED__inverse_kin__srv__InvKin_Request __declspec(deprecated)
#endif

namespace inverse_kin
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InvKin_Request_
{
  using Type = InvKin_Request_<ContainerAllocator>;

  explicit InvKin_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->quat_x = 0.0;
      this->quat_y = 0.0;
      this->quat_z = 0.0;
      this->quat_w = 0.0;
    }
  }

  explicit InvKin_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->quat_x = 0.0;
      this->quat_y = 0.0;
      this->quat_z = 0.0;
      this->quat_w = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _quat_x_type =
    double;
  _quat_x_type quat_x;
  using _quat_y_type =
    double;
  _quat_y_type quat_y;
  using _quat_z_type =
    double;
  _quat_z_type quat_z;
  using _quat_w_type =
    double;
  _quat_w_type quat_w;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__quat_x(
    const double & _arg)
  {
    this->quat_x = _arg;
    return *this;
  }
  Type & set__quat_y(
    const double & _arg)
  {
    this->quat_y = _arg;
    return *this;
  }
  Type & set__quat_z(
    const double & _arg)
  {
    this->quat_z = _arg;
    return *this;
  }
  Type & set__quat_w(
    const double & _arg)
  {
    this->quat_w = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inverse_kin::srv::InvKin_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const inverse_kin::srv::InvKin_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inverse_kin::srv::InvKin_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inverse_kin::srv::InvKin_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inverse_kin::srv::InvKin_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inverse_kin::srv::InvKin_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inverse_kin::srv::InvKin_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inverse_kin::srv::InvKin_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inverse_kin::srv::InvKin_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inverse_kin::srv::InvKin_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inverse_kin__srv__InvKin_Request
    std::shared_ptr<inverse_kin::srv::InvKin_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inverse_kin__srv__InvKin_Request
    std::shared_ptr<inverse_kin::srv::InvKin_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InvKin_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->quat_x != other.quat_x) {
      return false;
    }
    if (this->quat_y != other.quat_y) {
      return false;
    }
    if (this->quat_z != other.quat_z) {
      return false;
    }
    if (this->quat_w != other.quat_w) {
      return false;
    }
    return true;
  }
  bool operator!=(const InvKin_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InvKin_Request_

// alias to use template instance with default allocator
using InvKin_Request =
  inverse_kin::srv::InvKin_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace inverse_kin


#ifndef _WIN32
# define DEPRECATED__inverse_kin__srv__InvKin_Response __attribute__((deprecated))
#else
# define DEPRECATED__inverse_kin__srv__InvKin_Response __declspec(deprecated)
#endif

namespace inverse_kin
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InvKin_Response_
{
  using Type = InvKin_Response_<ContainerAllocator>;

  explicit InvKin_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->q1 = 0.0;
      this->q2 = 0.0;
      this->q3 = 0.0;
    }
  }

  explicit InvKin_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->q1 = 0.0;
      this->q2 = 0.0;
      this->q3 = 0.0;
    }
  }

  // field types and members
  using _q1_type =
    double;
  _q1_type q1;
  using _q2_type =
    double;
  _q2_type q2;
  using _q3_type =
    double;
  _q3_type q3;

  // setters for named parameter idiom
  Type & set__q1(
    const double & _arg)
  {
    this->q1 = _arg;
    return *this;
  }
  Type & set__q2(
    const double & _arg)
  {
    this->q2 = _arg;
    return *this;
  }
  Type & set__q3(
    const double & _arg)
  {
    this->q3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inverse_kin::srv::InvKin_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const inverse_kin::srv::InvKin_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inverse_kin::srv::InvKin_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inverse_kin::srv::InvKin_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inverse_kin::srv::InvKin_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inverse_kin::srv::InvKin_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inverse_kin::srv::InvKin_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inverse_kin::srv::InvKin_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inverse_kin::srv::InvKin_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inverse_kin::srv::InvKin_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inverse_kin__srv__InvKin_Response
    std::shared_ptr<inverse_kin::srv::InvKin_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inverse_kin__srv__InvKin_Response
    std::shared_ptr<inverse_kin::srv::InvKin_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InvKin_Response_ & other) const
  {
    if (this->q1 != other.q1) {
      return false;
    }
    if (this->q2 != other.q2) {
      return false;
    }
    if (this->q3 != other.q3) {
      return false;
    }
    return true;
  }
  bool operator!=(const InvKin_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InvKin_Response_

// alias to use template instance with default allocator
using InvKin_Response =
  inverse_kin::srv::InvKin_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace inverse_kin

namespace inverse_kin
{

namespace srv
{

struct InvKin
{
  using Request = inverse_kin::srv::InvKin_Request;
  using Response = inverse_kin::srv::InvKin_Response;
};

}  // namespace srv

}  // namespace inverse_kin

#endif  // INVERSE_KIN__SRV__DETAIL__INV_KIN__STRUCT_HPP_
