// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dwb_msgs:msg/Trajectory2D.idl
// generated code does not contain a copyright notice

#ifndef DWB_MSGS__MSG__DETAIL__TRAJECTORY2_D__TRAITS_HPP_
#define DWB_MSGS__MSG__DETAIL__TRAJECTORY2_D__TRAITS_HPP_

#include "dwb_msgs/msg/detail/trajectory2_d__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'velocity'
#include "nav_2d_msgs/msg/detail/twist2_d__traits.hpp"
// Member 'time_offsets'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"
// Member 'poses'
#include "geometry_msgs/msg/detail/pose2_d__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const dwb_msgs::msg::Trajectory2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_yaml(msg.velocity, out, indentation + 2);
  }

  // member: time_offsets
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.time_offsets.size() == 0) {
      out << "time_offsets: []\n";
    } else {
      out << "time_offsets:\n";
      for (auto item : msg.time_offsets) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poses.size() == 0) {
      out << "poses: []\n";
    } else {
      out << "poses:\n";
      for (auto item : msg.poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const dwb_msgs::msg::Trajectory2D & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<dwb_msgs::msg::Trajectory2D>()
{
  return "dwb_msgs::msg::Trajectory2D";
}

template<>
inline const char * name<dwb_msgs::msg::Trajectory2D>()
{
  return "dwb_msgs/msg/Trajectory2D";
}

template<>
struct has_fixed_size<dwb_msgs::msg::Trajectory2D>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dwb_msgs::msg::Trajectory2D>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dwb_msgs::msg::Trajectory2D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DWB_MSGS__MSG__DETAIL__TRAJECTORY2_D__TRAITS_HPP_
