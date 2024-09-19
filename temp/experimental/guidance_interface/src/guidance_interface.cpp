#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "guidance_interface/guidance_interface.hpp" // Include the custom global planner header

namespace guidance_interface // Define the namespace for the planner
{

// Configuration method to initialize the planner with parameters and resources
void GuidanceInterface::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, // Pointer to the parent lifecycle node
  std::string name, // Name of the planner
  std::shared_ptr<tf2_ros::Buffer> tf, // Transform buffer for coordinate transformations
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) // Costmap for navigation
{
  node_ = parent.lock(); // Lock the weak pointer to get the shared pointer to the node
  name_ = name; // Store the planner's name
  tf_ = tf; // Store the transform buffer
  costmap_ = costmap_ros->getCostmap(); // Get the costmap from the costmap ROS object
  global_frame_ = costmap_ros->getGlobalFrameID(); // Get the global frame ID from the costmap

  // Parameter initialization for interpolation resolution, with a default value of 0.1
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_); // Retrieve the parameter value
}

// Cleanup method called when the planner is deactivated
void GuidanceInterface::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type GuidanceInterface", // Log the cleanup process
    name_.c_str());
}

// Activation method called when the planner is activated
void GuidanceInterface::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type GuidanceInterface", // Log the activation process
    name_.c_str());
}

// Deactivation method called when the planner is deactivated
void GuidanceInterface::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type GuidanceInterface", // Log the deactivation process
    name_.c_str());
}

// Method to create a plan based on the start and goal poses
nav_msgs::msg::Path GuidanceInterface::createPlan(
  const geometry_msgs::msg::PoseStamped & start, // Starting pose
  const geometry_msgs::msg::PoseStamped & goal) // Goal pose
{
  nav_msgs::msg::Path global_path; // Initialize the path to be returned

  // Validate the frame of the start pose
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only accept start position from %s frame", // Log error if frame is incorrect
      global_frame_.c_str());
    return global_path; // Return empty path
  }

  // Validate the frame of the goal pose
  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only accept goal position from %s frame", // Log info if frame is incorrect
      global_frame_.c_str());
    return global_path; // Return empty path
  }

  global_path.poses.clear(); // Clear previous poses
  global_path.header.stamp = node_->now(); // Set the current timestamp
  global_path.header.frame_id = global_frame_; // Set the frame ID for the path

  // Calculate the number of interpolation steps based on distance and resolution
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x, // Calculate X distance
    goal.pose.position.y - start.pose.position.y) / // Calculate Y distance
    interpolation_resolution_; // Divide by resolution to get steps
  
  // Calculate increments in X and Y for each step
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  // Generate interpolated poses along the straight line from start to goal
  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose; // Create a new pose
    pose.pose.position.x = start.pose.position.x + x_increment * i; // Increment X
    pose.pose.position.y = start.pose.position.y + y_increment * i; // Increment Y
    pose.pose.position.z = 0.0; // Set Z to 0 for 2D navigation
    pose.pose.orientation.x = 0.0; // Orientation placeholder
    pose.pose.orientation.y = 0.0; // Orientation placeholder
    pose.pose.orientation.z = 0.0; // Orientation placeholder
    pose.pose.orientation.w = 1.0; // Set to default quaternion orientation
    pose.header.stamp = node_->now(); // Set the current timestamp
    pose.header.frame_id = global_frame_; // Set the frame ID for the pose
    global_path.poses.push_back(pose); // Add the pose to the path
  }

  // Add the goal pose to the path
  geometry_msgs::msg::PoseStamped goal_pose = goal; // Copy goal pose
  goal_pose.header.stamp = node_->now(); // Set the current timestamp
  goal_pose.header.frame_id = global_frame_; // Set the frame ID
  global_path.poses.push_back(goal_pose); // Add goal pose to the path

  return global_path; // Return the generated path
}

}  // namespace guidance_interface

#include "pluginlib/class_list_macros.hpp"
// Export the planner class for use with pluginlib
PLUGINLIB_EXPORT_CLASS(guidance_interface::GuidanceInterface, nav2_core::GlobalPlanner)

