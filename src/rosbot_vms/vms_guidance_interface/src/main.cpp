#include <memory>

#include "vms_guidance_interface/guidance_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/rclcpp.hpp"
#include "vms_guidance_interface/guidance_interface.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create a shared pointer to the node
    auto node = std::make_shared<vms_guidance_interface::GuidanceInterface>(rclcpp::NodeOptions());

    // Configure the node
    auto configure_result = node->configure();

    // Activate the node
    auto activate_result = node->activate();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Guidance Interface is now active and ready to accept goals.");

    // Spin the node to keep it alive and process callbacks
    rclcpp::spin(node->get_node_base_interface());

    // Shutdown and cleanup
    rclcpp::shutdown();
    return 0;
}
