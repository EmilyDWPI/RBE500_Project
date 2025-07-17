// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inverse_kin:srv/InvKin.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KIN__SRV__DETAIL__INV_KIN__TRAITS_HPP_
#define INVERSE_KIN__SRV__DETAIL__INV_KIN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inverse_kin/srv/detail/inv_kin__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace inverse_kin
{

namespace srv
{

inline void to_flow_style_yaml(
  const InvKin_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: quat_x
  {
    out << "quat_x: ";
    rosidl_generator_traits::value_to_yaml(msg.quat_x, out);
    out << ", ";
  }

  // member: quat_y
  {
    out << "quat_y: ";
    rosidl_generator_traits::value_to_yaml(msg.quat_y, out);
    out << ", ";
  }

  // member: quat_z
  {
    out << "quat_z: ";
    rosidl_generator_traits::value_to_yaml(msg.quat_z, out);
    out << ", ";
  }

  // member: quat_w
  {
    out << "quat_w: ";
    rosidl_generator_traits::value_to_yaml(msg.quat_w, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InvKin_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: quat_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quat_x: ";
    rosidl_generator_traits::value_to_yaml(msg.quat_x, out);
    out << "\n";
  }

  // member: quat_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quat_y: ";
    rosidl_generator_traits::value_to_yaml(msg.quat_y, out);
    out << "\n";
  }

  // member: quat_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quat_z: ";
    rosidl_generator_traits::value_to_yaml(msg.quat_z, out);
    out << "\n";
  }

  // member: quat_w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quat_w: ";
    rosidl_generator_traits::value_to_yaml(msg.quat_w, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InvKin_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace inverse_kin

namespace rosidl_generator_traits
{

[[deprecated("use inverse_kin::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const inverse_kin::srv::InvKin_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  inverse_kin::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inverse_kin::srv::to_yaml() instead")]]
inline std::string to_yaml(const inverse_kin::srv::InvKin_Request & msg)
{
  return inverse_kin::srv::to_yaml(msg);
}

template<>
inline const char * data_type<inverse_kin::srv::InvKin_Request>()
{
  return "inverse_kin::srv::InvKin_Request";
}

template<>
inline const char * name<inverse_kin::srv::InvKin_Request>()
{
  return "inverse_kin/srv/InvKin_Request";
}

template<>
struct has_fixed_size<inverse_kin::srv::InvKin_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<inverse_kin::srv::InvKin_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<inverse_kin::srv::InvKin_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace inverse_kin
{

namespace srv
{

inline void to_flow_style_yaml(
  const InvKin_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: q1
  {
    out << "q1: ";
    rosidl_generator_traits::value_to_yaml(msg.q1, out);
    out << ", ";
  }

  // member: q2
  {
    out << "q2: ";
    rosidl_generator_traits::value_to_yaml(msg.q2, out);
    out << ", ";
  }

  // member: q3
  {
    out << "q3: ";
    rosidl_generator_traits::value_to_yaml(msg.q3, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InvKin_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: q1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q1: ";
    rosidl_generator_traits::value_to_yaml(msg.q1, out);
    out << "\n";
  }

  // member: q2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q2: ";
    rosidl_generator_traits::value_to_yaml(msg.q2, out);
    out << "\n";
  }

  // member: q3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q3: ";
    rosidl_generator_traits::value_to_yaml(msg.q3, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InvKin_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace inverse_kin

namespace rosidl_generator_traits
{

[[deprecated("use inverse_kin::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const inverse_kin::srv::InvKin_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  inverse_kin::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inverse_kin::srv::to_yaml() instead")]]
inline std::string to_yaml(const inverse_kin::srv::InvKin_Response & msg)
{
  return inverse_kin::srv::to_yaml(msg);
}

template<>
inline const char * data_type<inverse_kin::srv::InvKin_Response>()
{
  return "inverse_kin::srv::InvKin_Response";
}

template<>
inline const char * name<inverse_kin::srv::InvKin_Response>()
{
  return "inverse_kin/srv/InvKin_Response";
}

template<>
struct has_fixed_size<inverse_kin::srv::InvKin_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<inverse_kin::srv::InvKin_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<inverse_kin::srv::InvKin_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<inverse_kin::srv::InvKin>()
{
  return "inverse_kin::srv::InvKin";
}

template<>
inline const char * name<inverse_kin::srv::InvKin>()
{
  return "inverse_kin/srv/InvKin";
}

template<>
struct has_fixed_size<inverse_kin::srv::InvKin>
  : std::integral_constant<
    bool,
    has_fixed_size<inverse_kin::srv::InvKin_Request>::value &&
    has_fixed_size<inverse_kin::srv::InvKin_Response>::value
  >
{
};

template<>
struct has_bounded_size<inverse_kin::srv::InvKin>
  : std::integral_constant<
    bool,
    has_bounded_size<inverse_kin::srv::InvKin_Request>::value &&
    has_bounded_size<inverse_kin::srv::InvKin_Response>::value
  >
{
};

template<>
struct is_service<inverse_kin::srv::InvKin>
  : std::true_type
{
};

template<>
struct is_service_request<inverse_kin::srv::InvKin_Request>
  : std::true_type
{
};

template<>
struct is_service_response<inverse_kin::srv::InvKin_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INVERSE_KIN__SRV__DETAIL__INV_KIN__TRAITS_HPP_
