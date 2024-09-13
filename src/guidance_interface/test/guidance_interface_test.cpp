#include <gtest/gtest.h>
#include <memory>
#include "guidance_interface/guidance_interface.hpp" // Include your GuidanceInterface header

class GuidanceInterfaceTest : public ::testing::Test {
protected:
    std::shared_ptr<guidance_interface::GuidanceInterface> planner_;

    void SetUp() override {
        planner_ = std::make_shared<guidance_interface::GuidanceInterface>(); // Create instance
    }
};

// Basic test case to check if the planner is initialized
TEST_F(GuidanceInterfaceTest, Initialization) {
    ASSERT_NE(planner_, nullptr); // Check if the planner is not null
}

// Main function to run the tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); // Initialize Google Test
    return RUN_ALL_TESTS(); // Run all tests
}

