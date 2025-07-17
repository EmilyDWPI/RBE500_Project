// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inverse_kin:srv/InvKin.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__SRV__DETAIL__INV_KIN__BUILDER_HPP_
#define INVERSE_KIN__SRV__DETAIL__INV_KIN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inverse_kin/srv/detail/inv_kin__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inverse_kin
{

namespace srv
{

namespace builder
{

class Init_InvKin_Request_quat_w
{
public:
  explicit Init_InvKin_Request_quat_w(::inverse_kin::srv::InvKin_Request & msg)
  : msg_(msg)
  {}
  ::inverse_kin::srv::InvKin_Request quat_w(::inverse_kin::srv::InvKin_Request::_quat_w_type arg)
  {
    msg_.quat_w = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inverse_kin::srv::InvKin_Request msg_;
};

class Init_InvKin_Request_quat_z
{
public:
  explicit Init_InvKin_Request_quat_z(::inverse_kin::srv::InvKin_Request & msg)
  : msg_(msg)
  {}
  Init_InvKin_Request_quat_w quat_z(::inverse_kin::srv::InvKin_Request::_quat_z_type arg)
  {
    msg_.quat_z = std::move(arg);
    return Init_InvKin_Request_quat_w(msg_);
  }

private:
  ::inverse_kin::srv::InvKin_Request msg_;
};

class Init_InvKin_Request_quat_y
{
public:
  explicit Init_InvKin_Request_quat_y(::inverse_kin::srv::InvKin_Request & msg)
  : msg_(msg)
  {}
  Init_InvKin_Request_quat_z quat_y(::inverse_kin::srv::InvKin_Request::_quat_y_type arg)
  {
    msg_.quat_y = std::move(arg);
    return Init_InvKin_Request_quat_z(msg_);
  }

private:
  ::inverse_kin::srv::InvKin_Request msg_;
};

class Init_InvKin_Request_quat_x
{
public:
  explicit Init_InvKin_Request_quat_x(::inverse_kin::srv::InvKin_Request & msg)
  : msg_(msg)
  {}
  Init_InvKin_Request_quat_y quat_x(::inverse_kin::srv::InvKin_Request::_quat_x_type arg)
  {
    msg_.quat_x = std::move(arg);
    return Init_InvKin_Request_quat_y(msg_);
  }

private:
  ::inverse_kin::srv::InvKin_Request msg_;
};

class Init_InvKin_Request_z
{
public:
  explicit Init_InvKin_Request_z(::inverse_kin::srv::InvKin_Request & msg)
  : msg_(msg)
  {}
  Init_InvKin_Request_quat_x z(::inverse_kin::srv::InvKin_Request::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_InvKin_Request_quat_x(msg_);
  }

private:
  ::inverse_kin::srv::InvKin_Request msg_;
};

class Init_InvKin_Request_y
{
public:
  explicit Init_InvKin_Request_y(::inverse_kin::srv::InvKin_Request & msg)
  : msg_(msg)
  {}
  Init_InvKin_Request_z y(::inverse_kin::srv::InvKin_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_InvKin_Request_z(msg_);
  }

private:
  ::inverse_kin::srv::InvKin_Request msg_;
};

class Init_InvKin_Request_x
{
public:
  Init_InvKin_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InvKin_Request_y x(::inverse_kin::srv::InvKin_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_InvKin_Request_y(msg_);
  }

private:
  ::inverse_kin::srv::InvKin_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::inverse_kin::srv::InvKin_Request>()
{
  return inverse_kin::srv::builder::Init_InvKin_Request_x();
}

}  // namespace inverse_kin


namespace inverse_kin
{

namespace srv
{

namespace builder
{

class Init_InvKin_Response_q3
{
public:
  explicit Init_InvKin_Response_q3(::inverse_kin::srv::InvKin_Response & msg)
  : msg_(msg)
  {}
  ::inverse_kin::srv::InvKin_Response q3(::inverse_kin::srv::InvKin_Response::_q3_type arg)
  {
    msg_.q3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inverse_kin::srv::InvKin_Response msg_;
};

class Init_InvKin_Response_q2
{
public:
  explicit Init_InvKin_Response_q2(::inverse_kin::srv::InvKin_Response & msg)
  : msg_(msg)
  {}
  Init_InvKin_Response_q3 q2(::inverse_kin::srv::InvKin_Response::_q2_type arg)
  {
    msg_.q2 = std::move(arg);
    return Init_InvKin_Response_q3(msg_);
  }

private:
  ::inverse_kin::srv::InvKin_Response msg_;
};

class Init_InvKin_Response_q1
{
public:
  Init_InvKin_Response_q1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InvKin_Response_q2 q1(::inverse_kin::srv::InvKin_Response::_q1_type arg)
  {
    msg_.q1 = std::move(arg);
    return Init_InvKin_Response_q2(msg_);
  }

private:
  ::inverse_kin::srv::InvKin_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::inverse_kin::srv::InvKin_Response>()
{
  return inverse_kin::srv::builder::Init_InvKin_Response_q1();
}

}  // namespace inverse_kin

#endif  // INVERSE_KIN__SRV__DETAIL__INV_KIN__BUILDER_HPP_
