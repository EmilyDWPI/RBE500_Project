// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inverse_kin:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__MSG__DETAIL__NUM__BUILDER_HPP_
#define INVERSE_KIN__MSG__DETAIL__NUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inverse_kin/msg/detail/num__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inverse_kin
{

namespace msg
{

namespace builder
{

class Init_Num_num
{
public:
  Init_Num_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::inverse_kin::msg::Num num(::inverse_kin::msg::Num::_num_type arg)
  {
    msg_.num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inverse_kin::msg::Num msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inverse_kin::msg::Num>()
{
  return inverse_kin::msg::builder::Init_Num_num();
}

}  // namespace inverse_kin

#endif  // INVERSE_KIN__MSG__DETAIL__NUM__BUILDER_HPP_
