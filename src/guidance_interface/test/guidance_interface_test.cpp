#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
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
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap", tf_buffer_); // Initialize the costmap

    planner_ = std::make_shared<guidance_interface::GuidanceInterface>(); // Create the planner

    // Configure the planner
    planner_->configure(node_->get_node_lifecycle_interface(), "test_planner", tf_buffer_, costmap_ros_);
  }

  void TearDown() override {
    planner_->cleanup(); // Cleanup after each test
    rclcpp::shutdown(); // Shutdown ROS 2
  }
};

TEST_F(GuidanceInterfaceTest, TestConfigure) {
  // Ensure the planner is correctly configured
  EXPECT_EQ(planner_->getGlobalFrameID(), costmap_ros_->getGlobalFrameID());
}

TEST_F(GuidanceInterfaceTest, TestCreatePlan) {
  // Create start and goal poses
  geometry_msgs::msg::PoseStamped start, goal;
  start.header.frame_id = costmap_ros_->getGlobalFrameID();
  goal.header.frame_id = costmap_ros_->getGlobalFrameID();
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;

  // Generate the plan
  nav_msgs::msg::Path path = planner_->createPlan(start, goal);

  // Validate that the path is non-empty and contains the correct number of points
  EXPECT_FALSE(path.poses.empty());
  EXPECT_EQ(path.poses.size(), 11); // Check the expected number of interpolated poses + goal pose
}

TEST_F(GuidanceInterfaceTest, TestInvalidFrame) {
  // Create start and goal poses with incorrect frame IDs
  geometry_msgs::msg::PoseStamped start, goal;
  start.header.frame_id = "invalid_frame";
  goal.header.frame_id = costmap_ros_->getGlobalFrameID();

  // Generate the plan with an invalid start frame
  nav_msgs::msg::Path path = planner_->createPlan(start, goal);

  // Validate that the path is empty due to the invalid frame
  EXPECT_TRUE(path.poses.empty());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

