#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "guidance_interface/guidance_interface.hpp"

class GuidanceInterfaceTest : public ::testing::Test {
protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<guidance_interface::GuidanceInterface> planner_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

    void SetUp() override {
        rclcpp::init(0, nullptr);  // Initialize ROS 2
        node_ = std::make_shared<rclcpp::Node>("test_node");  // Create a test node

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());  // Initialize the transform buffer
        tf2_ros::TransformListener tf_listener(*tf_buffer_);  // Create a listener for TF data

        // Initialize the costmap with the required parameters
        std::string name = "costmap";
        costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name);  // Initialize the costmap with just the name

        planner_ = std::make_shared<guidance_interface::GuidanceInterface>();  // Create the planner

        // Configure the planner
        planner_->configure(node_->get_node_base_interface(), "test_planner", tf_buffer_, costmap_ros_);
    }

    void TearDown() override {
        planner_->cleanup();  // Cleanup after each test
        rclcpp::shutdown();  // Shutdown ROS 2
    }
};

// Example test
TEST_F(GuidanceInterfaceTest, TestPlannerInitialization) {
    EXPECT_NE(planner_, nullptr);
    EXPECT_NE(costmap_ros_, nullptr);
    EXPECT_NE(tf_buffer_, nullptr);
}

// Main function to run tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

