// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__CONTROL__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_Control_Request_goal_theta
{
public:
  explicit Init_Control_Request_goal_theta(::custom_interfaces::srv::Control_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::Control_Request goal_theta(::custom_interfaces::srv::Control_Request::_goal_theta_type arg)
  {
    msg_.goal_theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::Control_Request msg_;
};

class Init_Control_Request_joint_name
{
public:
  Init_Control_Request_joint_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_Request_goal_theta joint_name(::custom_interfaces::srv::Control_Request::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_Control_Request_goal_theta(msg_);
  }

private:
  ::custom_interfaces::srv::Control_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::Control_Request>()
{
  return custom_interfaces::srv::builder::Init_Control_Request_joint_name();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_Control_Response_effort
{
public:
  explicit Init_Control_Response_effort(::custom_interfaces::srv::Control_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::Control_Response effort(::custom_interfaces::srv::Control_Response::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::Control_Response msg_;
};

class Init_Control_Response_joint_name
{
public:
  Init_Control_Response_joint_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_Response_effort joint_name(::custom_interfaces::srv::Control_Response::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_Control_Response_effort(msg_);
  }

private:
  ::custom_interfaces::srv::Control_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::Control_Response>()
{
  return custom_interfaces::srv::builder::Init_Control_Response_joint_name();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__CONTROL__BUILDER_HPP_
