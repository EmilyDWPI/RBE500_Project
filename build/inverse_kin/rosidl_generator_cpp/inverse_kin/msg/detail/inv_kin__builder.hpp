// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inverse_kin:msg/InvKin.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__MSG__DETAIL__INV_KIN__BUILDER_HPP_
#define INVERSE_KIN__MSG__DETAIL__INV_KIN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inverse_kin/msg/detail/inv_kin__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inverse_kin
{

namespace msg
{

namespace builder
{

class Init_InvKin_ee_angl
{
public:
  explicit Init_InvKin_ee_angl(::inverse_kin::msg::InvKin & msg)
  : msg_(msg)
  {}
  ::inverse_kin::msg::InvKin ee_angl(::inverse_kin::msg::InvKin::_ee_angl_type arg)
  {
    msg_.ee_angl = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inverse_kin::msg::InvKin msg_;
};

class Init_InvKin_ee_pos
{
public:
  explicit Init_InvKin_ee_pos(::inverse_kin::msg::InvKin & msg)
  : msg_(msg)
  {}
  Init_InvKin_ee_angl ee_pos(::inverse_kin::msg::InvKin::_ee_pos_type arg)
  {
    msg_.ee_pos = std::move(arg);
    return Init_InvKin_ee_angl(msg_);
  }

private:
  ::inverse_kin::msg::InvKin msg_;
};

class Init_InvKin_scara_pose
{
public:
  Init_InvKin_scara_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InvKin_ee_pos scara_pose(::inverse_kin::msg::InvKin::_scara_pose_type arg)
  {
    msg_.scara_pose = std::move(arg);
    return Init_InvKin_ee_pos(msg_);
  }

private:
  ::inverse_kin::msg::InvKin msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inverse_kin::msg::InvKin>()
{
  return inverse_kin::msg::builder::Init_InvKin_scara_pose();
}

}  // namespace inverse_kin

#endif  // INVERSE_KIN__MSG__DETAIL__INV_KIN__BUILDER_HPP_
