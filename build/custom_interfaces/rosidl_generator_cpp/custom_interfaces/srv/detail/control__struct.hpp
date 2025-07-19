// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__CONTROL__STRUCT_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__Control_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__Control_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Control_Request_
{
  using Type = Control_Request_<ContainerAllocator>;

  explicit Control_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->goal_theta = 0.0;
    }
  }

  explicit Control_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->goal_theta = 0.0;
    }
  }

  // field types and members
  using _joint_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_name_type joint_name;
  using _goal_theta_type =
    double;
  _goal_theta_type goal_theta;

  // setters for named parameter idiom
  Type & set__joint_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->joint_name = _arg;
    return *this;
  }
  Type & set__goal_theta(
    const double & _arg)
  {
    this->goal_theta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::Control_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::Control_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::Control_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::Control_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::Control_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::Control_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::Control_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::Control_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::Control_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::Control_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__Control_Request
    std::shared_ptr<custom_interfaces::srv::Control_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__Control_Request
    std::shared_ptr<custom_interfaces::srv::Control_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Control_Request_ & other) const
  {
    if (this->joint_name != other.joint_name) {
      return false;
    }
    if (this->goal_theta != other.goal_theta) {
      return false;
    }
    return true;
  }
  bool operator!=(const Control_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Control_Request_

// alias to use template instance with default allocator
using Control_Request =
  custom_interfaces::srv::Control_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__Control_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__Control_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Control_Response_
{
  using Type = Control_Response_<ContainerAllocator>;

  explicit Control_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->effort = 0.0;
    }
  }

  explicit Control_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->effort = 0.0;
    }
  }

  // field types and members
  using _joint_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_name_type joint_name;
  using _effort_type =
    double;
  _effort_type effort;

  // setters for named parameter idiom
  Type & set__joint_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->joint_name = _arg;
    return *this;
  }
  Type & set__effort(
    const double & _arg)
  {
    this->effort = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::Control_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::Control_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::Control_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::Control_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::Control_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::Control_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::Control_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::Control_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::Control_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::Control_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__Control_Response
    std::shared_ptr<custom_interfaces::srv::Control_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__Control_Response
    std::shared_ptr<custom_interfaces::srv::Control_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Control_Response_ & other) const
  {
    if (this->joint_name != other.joint_name) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    return true;
  }
  bool operator!=(const Control_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Control_Response_

// alias to use template instance with default allocator
using Control_Response =
  custom_interfaces::srv::Control_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace srv
{

struct Control
{
  using Request = custom_interfaces::srv::Control_Request;
  using Response = custom_interfaces::srv::Control_Response;
};

}  // namespace srv

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__CONTROL__STRUCT_HPP_
