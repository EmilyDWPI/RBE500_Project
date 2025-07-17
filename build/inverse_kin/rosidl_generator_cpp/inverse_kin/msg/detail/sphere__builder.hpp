// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inverse_kin:msg/Sphere.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__MSG__DETAIL__SPHERE__BUILDER_HPP_
#define INVERSE_KIN__MSG__DETAIL__SPHERE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inverse_kin/msg/detail/sphere__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inverse_kin
{

namespace msg
{

namespace builder
{

class Init_Sphere_radius
{
public:
  explicit Init_Sphere_radius(::inverse_kin::msg::Sphere & msg)
  : msg_(msg)
  {}
  ::inverse_kin::msg::Sphere radius(::inverse_kin::msg::Sphere::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inverse_kin::msg::Sphere msg_;
};

class Init_Sphere_center
{
public:
  Init_Sphere_center()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sphere_radius center(::inverse_kin::msg::Sphere::_center_type arg)
  {
    msg_.center = std::move(arg);
    return Init_Sphere_radius(msg_);
  }

private:
  ::inverse_kin::msg::Sphere msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inverse_kin::msg::Sphere>()
{
  return inverse_kin::msg::builder::Init_Sphere_center();
}

}  // namespace inverse_kin

#endif  // INVERSE_KIN__MSG__DETAIL__SPHERE__BUILDER_HPP_
