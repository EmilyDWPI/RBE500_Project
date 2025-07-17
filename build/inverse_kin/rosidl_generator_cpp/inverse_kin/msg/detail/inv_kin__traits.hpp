// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inverse_kin:msg/InvKin.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__MSG__DETAIL__INV_KIN__TRAITS_HPP_
#define INVERSE_KIN__MSG__DETAIL__INV_KIN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inverse_kin/msg/detail/inv_kin__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'scara_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'ee_pos'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'ee_angl'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace inverse_kin
{

namespace msg
{

inline void to_flow_style_yaml(
  const InvKin & msg,
  std::ostream & out)
{
  out << "{";
  // member: scara_pose
  {
    out << "scara_pose: ";
    to_flow_style_yaml(msg.scara_pose, out);
    out << ", ";
  }

  // member: ee_pos
  {
    out << "ee_pos: ";
    to_flow_style_yaml(msg.ee_pos, out);
    out << ", ";
  }

  // member: ee_angl
  {
    out << "ee_angl: ";
    to_flow_style_yaml(msg.ee_angl, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InvKin & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: scara_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scara_pose:\n";
    to_block_style_yaml(msg.scara_pose, out, indentation + 2);
  }

  // member: ee_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ee_pos:\n";
    to_block_style_yaml(msg.ee_pos, out, indentation + 2);
  }

  // member: ee_angl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ee_angl:\n";
    to_block_style_yaml(msg.ee_angl, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InvKin & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace inverse_kin

namespace rosidl_generator_traits
{

[[deprecated("use inverse_kin::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const inverse_kin::msg::InvKin & msg,
  std::ostream & out, size_t indentation = 0)
{
  inverse_kin::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inverse_kin::msg::to_yaml() instead")]]
inline std::string to_yaml(const inverse_kin::msg::InvKin & msg)
{
  return inverse_kin::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inverse_kin::msg::InvKin>()
{
  return "inverse_kin::msg::InvKin";
}

template<>
inline const char * name<inverse_kin::msg::InvKin>()
{
  return "inverse_kin/msg/InvKin";
}

template<>
struct has_fixed_size<inverse_kin::msg::InvKin>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Quaternion>::value> {};

template<>
struct has_bounded_size<inverse_kin::msg::InvKin>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Quaternion>::value> {};

template<>
struct is_message<inverse_kin::msg::InvKin>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INVERSE_KIN__MSG__DETAIL__INV_KIN__TRAITS_HPP_
