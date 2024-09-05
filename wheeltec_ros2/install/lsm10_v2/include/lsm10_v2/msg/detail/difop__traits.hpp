// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lsm10_v2:msg/Difop.idl
// generated code does not contain a copyright notice

#ifndef LSM10_V2__MSG__DETAIL__DIFOP__TRAITS_HPP_
#define LSM10_V2__MSG__DETAIL__DIFOP__TRAITS_HPP_

#include "lsm10_v2/msg/detail/difop__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const lsm10_v2::msg::Difop & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motorspeed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motorspeed: ";
    value_to_yaml(msg.motorspeed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const lsm10_v2::msg::Difop & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<lsm10_v2::msg::Difop>()
{
  return "lsm10_v2::msg::Difop";
}

template<>
inline const char * name<lsm10_v2::msg::Difop>()
{
  return "lsm10_v2/msg/Difop";
}

template<>
struct has_fixed_size<lsm10_v2::msg::Difop>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<lsm10_v2::msg::Difop>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<lsm10_v2::msg::Difop>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LSM10_V2__MSG__DETAIL__DIFOP__TRAITS_HPP_
