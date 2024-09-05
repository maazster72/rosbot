// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dwb_msgs:msg/TrajectoryScore.idl
// generated code does not contain a copyright notice

#ifndef DWB_MSGS__MSG__DETAIL__TRAJECTORY_SCORE__TRAITS_HPP_
#define DWB_MSGS__MSG__DETAIL__TRAJECTORY_SCORE__TRAITS_HPP_

#include "dwb_msgs/msg/detail/trajectory_score__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'traj'
#include "dwb_msgs/msg/detail/trajectory2_d__traits.hpp"
// Member 'scores'
#include "dwb_msgs/msg/detail/critic_score__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const dwb_msgs::msg::TrajectoryScore & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: traj
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "traj:\n";
    to_yaml(msg.traj, out, indentation + 2);
  }

  // member: scores
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.scores.size() == 0) {
      out << "scores: []\n";
    } else {
      out << "scores:\n";
      for (auto item : msg.scores) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: total
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total: ";
    value_to_yaml(msg.total, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const dwb_msgs::msg::TrajectoryScore & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<dwb_msgs::msg::TrajectoryScore>()
{
  return "dwb_msgs::msg::TrajectoryScore";
}

template<>
inline const char * name<dwb_msgs::msg::TrajectoryScore>()
{
  return "dwb_msgs/msg/TrajectoryScore";
}

template<>
struct has_fixed_size<dwb_msgs::msg::TrajectoryScore>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dwb_msgs::msg::TrajectoryScore>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dwb_msgs::msg::TrajectoryScore>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DWB_MSGS__MSG__DETAIL__TRAJECTORY_SCORE__TRAITS_HPP_
