// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turn_on_wheeltec_robot:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__TRAITS_HPP_
#define TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__TRAITS_HPP_

#include "turn_on_wheeltec_robot/msg/detail/position__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const turn_on_wheeltec_robot::msg::Position & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angle_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_x: ";
    value_to_yaml(msg.angle_x, out);
    out << "\n";
  }

  // member: angle_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_y: ";
    value_to_yaml(msg.angle_y, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const turn_on_wheeltec_robot::msg::Position & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<turn_on_wheeltec_robot::msg::Position>()
{
  return "turn_on_wheeltec_robot::msg::Position";
}

template<>
inline const char * name<turn_on_wheeltec_robot::msg::Position>()
{
  return "turn_on_wheeltec_robot/msg/Position";
}

template<>
struct has_fixed_size<turn_on_wheeltec_robot::msg::Position>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turn_on_wheeltec_robot::msg::Position>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turn_on_wheeltec_robot::msg::Position>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURN_ON_WHEELTEC_ROBOT__MSG__DETAIL__POSITION__TRAITS_HPP_
