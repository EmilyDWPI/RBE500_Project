// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inverse_kin:srv/AddThreeInts.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__SRV__DETAIL__ADD_THREE_INTS__BUILDER_HPP_
#define INVERSE_KIN__SRV__DETAIL__ADD_THREE_INTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inverse_kin/srv/detail/add_three_ints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inverse_kin
{

namespace srv
{

namespace builder
{

class Init_AddThreeInts_Request_c
{
public:
  explicit Init_AddThreeInts_Request_c(::inverse_kin::srv::AddThreeInts_Request & msg)
  : msg_(msg)
  {}
  ::inverse_kin::srv::AddThreeInts_Request c(::inverse_kin::srv::AddThreeInts_Request::_c_type arg)
  {
    msg_.c = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inverse_kin::srv::AddThreeInts_Request msg_;
};

class Init_AddThreeInts_Request_b
{
public:
  explicit Init_AddThreeInts_Request_b(::inverse_kin::srv::AddThreeInts_Request & msg)
  : msg_(msg)
  {}
  Init_AddThreeInts_Request_c b(::inverse_kin::srv::AddThreeInts_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_AddThreeInts_Request_c(msg_);
  }

private:
  ::inverse_kin::srv::AddThreeInts_Request msg_;
};

class Init_AddThreeInts_Request_a
{
public:
  Init_AddThreeInts_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AddThreeInts_Request_b a(::inverse_kin::srv::AddThreeInts_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_AddThreeInts_Request_b(msg_);
  }

private:
  ::inverse_kin::srv::AddThreeInts_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::inverse_kin::srv::AddThreeInts_Request>()
{
  return inverse_kin::srv::builder::Init_AddThreeInts_Request_a();
}

}  // namespace inverse_kin


namespace inverse_kin
{

namespace srv
{

namespace builder
{

class Init_AddThreeInts_Response_sum
{
public:
  Init_AddThreeInts_Response_sum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::inverse_kin::srv::AddThreeInts_Response sum(::inverse_kin::srv::AddThreeInts_Response::_sum_type arg)
  {
    msg_.sum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inverse_kin::srv::AddThreeInts_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::inverse_kin::srv::AddThreeInts_Response>()
{
  return inverse_kin::srv::builder::Init_AddThreeInts_Response_sum();
}

}  // namespace inverse_kin

#endif  // INVERSE_KIN__SRV__DETAIL__ADD_THREE_INTS__BUILDER_HPP_
