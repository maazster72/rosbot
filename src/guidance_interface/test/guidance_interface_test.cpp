#include <gtest/gtest.h>
#include <memory>
#include "guidance_interface/guidance_interface.hpp" // Include your GuidanceInterface header

class GuidanceInterfaceTest : public ::testing::Test {
protected:
    std::shared_ptr<guidance_interface::GuidanceInterface> planner_;

    void SetUp() override {
        planner_ = std::make_shared<guidance_interface::GuidanceInterface>(); // Create instance
        // Simulate configuration
        rclcpp::NodeOptions options;
        auto node = std::make_shared<rclcpp::Node>("test_node", options);
        planner_->configure(node->get_node_base_interface(), "test_planner", nullptr, nullptr);
    }
};

// Basic test case to check if the planner is initialised
TEST_F(GuidanceInterfaceTest, Initialisation) {
    ASSERT_NE(planner_, nullptr); // Check if the planner is not null
}

// Test case for empty plan when frames do not match
TEST_F(GuidanceInterfaceTest, CreatePlanFrameMismatch) {
    // Set up start and goal poses with mismatched frames
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = "FrameOne";
    start.pose.position.x = 0.0;
    start.pose.position.y = 0.0;

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "NotFrameOne"; // Different frame
    goal.pose.position.x = 2.0;
    goal.pose.position.y = 2.0;

    // Call createPlan
    nav_msgs::msg::Path plan = planner_->createPlan(start, goal);

    // Assert that the plan is empty
    ASSERT_TRUE(plan.poses.empty());
}

// Main function to run the tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); // Initialize Google Test
    return RUN_ALL_TESTS(); // Run all tests
}

