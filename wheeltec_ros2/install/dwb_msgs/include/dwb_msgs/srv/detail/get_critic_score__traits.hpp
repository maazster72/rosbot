// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dwb_msgs:srv/GetCriticScore.idl
// generated code does not contain a copyright notice

#ifndef DWB_MSGS__SRV__DETAIL__GET_CRITIC_SCORE__TRAITS_HPP_
#define DWB_MSGS__SRV__DETAIL__GET_CRITIC_SCORE__TRAITS_HPP_

#include "dwb_msgs/srv/detail/get_critic_score__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "nav_2d_msgs/msg/detail/pose2_d_stamped__traits.hpp"
// Member 'velocity'
#include "nav_2d_msgs/msg/detail/twist2_d__traits.hpp"
// Member 'global_plan'
#include "nav_2d_msgs/msg/detail/path2_d__traits.hpp"
// Member 'traj'
#include "dwb_msgs/msg/detail/trajectory2_d__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const dwb_msgs::srv::GetCriticScore_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_yaml(msg.pose, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_yaml(msg.velocity, out, indentation + 2);
  }

  // member: global_plan
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "global_plan:\n";
    to_yaml(msg.global_plan, out, indentation + 2);
  }

  // member: traj
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "traj:\n";
    to_yaml(msg.traj, out, indentation + 2);
  }

  // member: critic_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "critic_name: ";
    value_to_yaml(msg.critic_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const dwb_msgs::srv::GetCriticScore_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<dwb_msgs::srv::GetCriticScore_Request>()
{
  return "dwb_msgs::srv::GetCriticScore_Request";
}

template<>
inline const char * name<dwb_msgs::srv::GetCriticScore_Request>()
{
  return "dwb_msgs/srv/GetCriticScore_Request";
}

template<>
struct has_fixed_size<dwb_msgs::srv::GetCriticScore_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dwb_msgs::srv::GetCriticScore_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dwb_msgs::srv::GetCriticScore_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'score'
#include "dwb_msgs/msg/detail/critic_score__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const dwb_msgs::srv::GetCriticScore_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "score:\n";
    to_yaml(msg.score, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const dwb_msgs::srv::GetCriticScore_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<dwb_msgs::srv::GetCriticScore_Response>()
{
  return "dwb_msgs::srv::GetCriticScore_Response";
}

template<>
inline const char * name<dwb_msgs::srv::GetCriticScore_Response>()
{
  return "dwb_msgs/srv/GetCriticScore_Response";
}

template<>
struct has_fixed_size<dwb_msgs::srv::GetCriticScore_Response>
  : std::integral_constant<bool, has_fixed_size<dwb_msgs::msg::CriticScore>::value> {};

template<>
struct has_bounded_size<dwb_msgs::srv::GetCriticScore_Response>
  : std::integral_constant<bool, has_bounded_size<dwb_msgs::msg::CriticScore>::value> {};

template<>
struct is_message<dwb_msgs::srv::GetCriticScore_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dwb_msgs::srv::GetCriticScore>()
{
  return "dwb_msgs::srv::GetCriticScore";
}

template<>
inline const char * name<dwb_msgs::srv::GetCriticScore>()
{
  return "dwb_msgs/srv/GetCriticScore";
}

template<>
struct has_fixed_size<dwb_msgs::srv::GetCriticScore>
  : std::integral_constant<
    bool,
    has_fixed_size<dwb_msgs::srv::GetCriticScore_Request>::value &&
    has_fixed_size<dwb_msgs::srv::GetCriticScore_Response>::value
  >
{
};

template<>
struct has_bounded_size<dwb_msgs::srv::GetCriticScore>
  : std::integral_constant<
    bool,
    has_bounded_size<dwb_msgs::srv::GetCriticScore_Request>::value &&
    has_bounded_size<dwb_msgs::srv::GetCriticScore_Response>::value
  >
{
};

template<>
struct is_service<dwb_msgs::srv::GetCriticScore>
  : std::true_type
{
};

template<>
struct is_service_request<dwb_msgs::srv::GetCriticScore_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dwb_msgs::srv::GetCriticScore_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DWB_MSGS__SRV__DETAIL__GET_CRITIC_SCORE__TRAITS_HPP_
