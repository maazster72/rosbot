// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dwb_msgs:srv/ScoreTrajectory.idl
// generated code does not contain a copyright notice

#ifndef DWB_MSGS__SRV__DETAIL__SCORE_TRAJECTORY__TRAITS_HPP_
#define DWB_MSGS__SRV__DETAIL__SCORE_TRAJECTORY__TRAITS_HPP_

#include "dwb_msgs/srv/detail/score_trajectory__struct.hpp"
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
  const dwb_msgs::srv::ScoreTrajectory_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const dwb_msgs::srv::ScoreTrajectory_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<dwb_msgs::srv::ScoreTrajectory_Request>()
{
  return "dwb_msgs::srv::ScoreTrajectory_Request";
}

template<>
inline const char * name<dwb_msgs::srv::ScoreTrajectory_Request>()
{
  return "dwb_msgs/srv/ScoreTrajectory_Request";
}

template<>
struct has_fixed_size<dwb_msgs::srv::ScoreTrajectory_Request>
  : std::integral_constant<bool, has_fixed_size<dwb_msgs::msg::Trajectory2D>::value && has_fixed_size<nav_2d_msgs::msg::Path2D>::value && has_fixed_size<nav_2d_msgs::msg::Pose2DStamped>::value && has_fixed_size<nav_2d_msgs::msg::Twist2D>::value> {};

template<>
struct has_bounded_size<dwb_msgs::srv::ScoreTrajectory_Request>
  : std::integral_constant<bool, has_bounded_size<dwb_msgs::msg::Trajectory2D>::value && has_bounded_size<nav_2d_msgs::msg::Path2D>::value && has_bounded_size<nav_2d_msgs::msg::Pose2DStamped>::value && has_bounded_size<nav_2d_msgs::msg::Twist2D>::value> {};

template<>
struct is_message<dwb_msgs::srv::ScoreTrajectory_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'score'
#include "dwb_msgs/msg/detail/trajectory_score__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const dwb_msgs::srv::ScoreTrajectory_Response & msg,
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

inline std::string to_yaml(const dwb_msgs::srv::ScoreTrajectory_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<dwb_msgs::srv::ScoreTrajectory_Response>()
{
  return "dwb_msgs::srv::ScoreTrajectory_Response";
}

template<>
inline const char * name<dwb_msgs::srv::ScoreTrajectory_Response>()
{
  return "dwb_msgs/srv/ScoreTrajectory_Response";
}

template<>
struct has_fixed_size<dwb_msgs::srv::ScoreTrajectory_Response>
  : std::integral_constant<bool, has_fixed_size<dwb_msgs::msg::TrajectoryScore>::value> {};

template<>
struct has_bounded_size<dwb_msgs::srv::ScoreTrajectory_Response>
  : std::integral_constant<bool, has_bounded_size<dwb_msgs::msg::TrajectoryScore>::value> {};

template<>
struct is_message<dwb_msgs::srv::ScoreTrajectory_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dwb_msgs::srv::ScoreTrajectory>()
{
  return "dwb_msgs::srv::ScoreTrajectory";
}

template<>
inline const char * name<dwb_msgs::srv::ScoreTrajectory>()
{
  return "dwb_msgs/srv/ScoreTrajectory";
}

template<>
struct has_fixed_size<dwb_msgs::srv::ScoreTrajectory>
  : std::integral_constant<
    bool,
    has_fixed_size<dwb_msgs::srv::ScoreTrajectory_Request>::value &&
    has_fixed_size<dwb_msgs::srv::ScoreTrajectory_Response>::value
  >
{
};

template<>
struct has_bounded_size<dwb_msgs::srv::ScoreTrajectory>
  : std::integral_constant<
    bool,
    has_bounded_size<dwb_msgs::srv::ScoreTrajectory_Request>::value &&
    has_bounded_size<dwb_msgs::srv::ScoreTrajectory_Response>::value
  >
{
};

template<>
struct is_service<dwb_msgs::srv::ScoreTrajectory>
  : std::true_type
{
};

template<>
struct is_service_request<dwb_msgs::srv::ScoreTrajectory_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dwb_msgs::srv::ScoreTrajectory_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DWB_MSGS__SRV__DETAIL__SCORE_TRAJECTORY__TRAITS_HPP_
