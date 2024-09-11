#ifndef GUIDANCE_INTERFACE__GUIDANCE_INTERFACE_HPP_
#define GUIDANCE_INTERFACE__GUIDANCE_INTERFACE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h" // Include tf2_ros for tf buffer

namespace guidance_interface {

class GuidanceInterface : public nav2_core::GlobalPlanner {
public:
  GuidanceInterface() = default;
  ~GuidanceInterface() = default; 

  // Plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,  // Pass name as const reference for better performance
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // Plugin cleanup
  void cleanup() override;

  // Plugin activate
  void activate() override;

  // Plugin deactivate
  void deactivate() override;

  // This method creates a path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;  // Ensure the tf buffer is a shared pointer

  // Node pointer
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;  // Changed to SharedPtr

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;  // Pointer to Costmap2D

  // The global frame of the costmap
  std::string global_frame_;  // Make sure to have only the global frame variable
  std::string name_;  // Name of the planner

  double interpolation_resolution_;  // Resolution for interpolation
};

}  // namespace guidance_interface

#endif  // GUIDANCE_INTERFACE__GUIDANCE_INTERFACE_HPP_

