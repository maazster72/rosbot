#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp> // Include for Costmap2DROS
#include "guidance_interface/guidance_interface.hpp" // Include the guidance interface

class GuidanceInterfaceTest : public ::testing::Test {
protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<guidance_interface::GuidanceInterface> planner_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  void SetUp() override {
    rclcpp::init(0, nullptr); // Initialize ROS 2
    node_ = std::make_shared<rclcpp::Node>("test_node"); // Create a test node
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock()); // Initialize the transform buffer

    // Initialize the costmap with the required parameters
    std::string name = "costmap";
    std::string global_frame = "map"; // Assuming "map" as the global frame; adjust as necessary
    std::string robot_base_frame = "base_link"; // Assuming "base_link" as the robot base frame; adjust as necessary
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name, global_frame, robot_base_frame, tf_buffer_); // Initialize the costmap

    planner_ = std::make_shared<guidance_interface::GuidanceInterface>(); // Create the planner

    // Configure the planner
    planner_->configure(node_->get_node_base_interface(), "test_planner", tf_buffer_, costmap_ros_);
  }

  void TearDown() override {
    planner_->cleanup(); // Cleanup after each test
    rclcpp::shutdown(); // Shutdown ROS 2
  }
};

// Add tests here

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

