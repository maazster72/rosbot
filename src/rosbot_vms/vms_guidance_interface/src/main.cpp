#include <memory>

#include "vms_guidance_interface/guidance_interface.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<vms_guidance_interface::GuidanceInterface>(options);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}