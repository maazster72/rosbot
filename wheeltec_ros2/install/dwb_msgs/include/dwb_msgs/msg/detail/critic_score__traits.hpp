// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dwb_msgs:msg/CriticScore.idl
// generated code does not contain a copyright notice

#ifndef DWB_MSGS__MSG__DETAIL__CRITIC_SCORE__TRAITS_HPP_
#define DWB_MSGS__MSG__DETAIL__CRITIC_SCORE__TRAITS_HPP_

#include "dwb_msgs/msg/detail/critic_score__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const dwb_msgs::msg::CriticScore & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: raw_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "raw_score: ";
    value_to_yaml(msg.raw_score, out);
    out << "\n";
  }

  // member: scale
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scale: ";
    value_to_yaml(msg.scale, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const dwb_msgs::msg::CriticScore & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<dwb_msgs::msg::CriticScore>()
{
  return "dwb_msgs::msg::CriticScore";
}

template<>
inline const char * name<dwb_msgs::msg::CriticScore>()
{
  return "dwb_msgs/msg/CriticScore";
}

template<>
struct has_fixed_size<dwb_msgs::msg::CriticScore>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dwb_msgs::msg::CriticScore>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dwb_msgs::msg::CriticScore>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DWB_MSGS__MSG__DETAIL__CRITIC_SCORE__TRAITS_HPP_
