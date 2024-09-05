// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dwb_msgs:msg/LocalPlanEvaluation.idl
// generated code does not contain a copyright notice

#ifndef DWB_MSGS__MSG__DETAIL__LOCAL_PLAN_EVALUATION__TRAITS_HPP_
#define DWB_MSGS__MSG__DETAIL__LOCAL_PLAN_EVALUATION__TRAITS_HPP_

#include "dwb_msgs/msg/detail/local_plan_evaluation__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'twists'
#include "dwb_msgs/msg/detail/trajectory_score__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const dwb_msgs::msg::LocalPlanEvaluation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_yaml(msg.header, out, indentation + 2);
  }

  // member: twists
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.twists.size() == 0) {
      out << "twists: []\n";
    } else {
      out << "twists:\n";
      for (auto item : msg.twists) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: best_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "best_index: ";
    value_to_yaml(msg.best_index, out);
    out << "\n";
  }

  // member: worst_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "worst_index: ";
    value_to_yaml(msg.worst_index, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const dwb_msgs::msg::LocalPlanEvaluation & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<dwb_msgs::msg::LocalPlanEvaluation>()
{
  return "dwb_msgs::msg::LocalPlanEvaluation";
}

template<>
inline const char * name<dwb_msgs::msg::LocalPlanEvaluation>()
{
  return "dwb_msgs/msg/LocalPlanEvaluation";
}

template<>
struct has_fixed_size<dwb_msgs::msg::LocalPlanEvaluation>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dwb_msgs::msg::LocalPlanEvaluation>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dwb_msgs::msg::LocalPlanEvaluation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DWB_MSGS__MSG__DETAIL__LOCAL_PLAN_EVALUATION__TRAITS_HPP_
