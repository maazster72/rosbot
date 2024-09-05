// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wheeltec_robot_msg:msg/Data.idl
// generated code does not contain a copyright notice

#ifndef WHEELTEC_ROBOT_MSG__MSG__DETAIL__DATA__TRAITS_HPP_
#define WHEELTEC_ROBOT_MSG__MSG__DETAIL__DATA__TRAITS_HPP_

#include "wheeltec_robot_msg/msg/detail/data__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const wheeltec_robot_msg::msg::Data & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    value_to_yaml(msg.z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const wheeltec_robot_msg::msg::Data & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<wheeltec_robot_msg::msg::Data>()
{
  return "wheeltec_robot_msg::msg::Data";
}

template<>
inline const char * name<wheeltec_robot_msg::msg::Data>()
{
  return "wheeltec_robot_msg/msg/Data";
}

template<>
struct has_fixed_size<wheeltec_robot_msg::msg::Data>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<wheeltec_robot_msg::msg::Data>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<wheeltec_robot_msg::msg::Data>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WHEELTEC_ROBOT_MSG__MSG__DETAIL__DATA__TRAITS_HPP_
