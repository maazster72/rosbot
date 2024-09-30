#include "vms_simple_translator/simple_translator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

namespace vms_simple_translator
{

// Member function to convert latitude and longitude to Cartesian coordinates
void SimpleTranslator::latLongToCartesian(double latitude, double longitude, double &x, double &y)
{


    // Simple equirectangular projection
    x = latitude;
    y = longitude;
}

void SimpleTranslator::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & /*parent*/,
    std::string /*name*/)
{
    // Initialize the clock
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    RCLCPP_INFO(rclcpp::get_logger("SimpleTranslator"), "Configuring Simple Translator");
}

void SimpleTranslator::cleanup()
{
    // Cleanup resources, if any
    RCLCPP_INFO(rclcpp::get_logger("SimpleTranslator"), "Cleaning up Simple Translator");
}

void SimpleTranslator::activate()
{
    // Code to activate translator
    RCLCPP_INFO(rclcpp::get_logger("SimpleTranslator"), "Activating Simple Translator");
}

void SimpleTranslator::deactivate()
{
    // Code to deactivate translator
    RCLCPP_INFO(rclcpp::get_logger("SimpleTranslator"), "Deactivating Simple Translator");
}

nav_msgs::msg::Path SimpleTranslator::convertRoute(const vms_msgs::msg::Route & route)
{
    nav_msgs::msg::Path path;
    path.header.stamp = clock_->now(); // Set to current time using the clock_
    path.header.frame_id = "odom"; // Set to the appropriate frame

    // Convert each RoutePoint to Cartesian coordinates
    for (const auto & point : route.routepoints) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;

        // Convert latitude and longitude to Cartesian coordinates
        latLongToCartesian(point.latitude, point.longitude, pose.pose.position.x, pose.pose.position.y);
        pose.pose.position.z = point.altitude;

        // Set orientation if needed (default to no rotation)
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
    }

    return path;
}

} // namespace vms_simple_translator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vms_simple_translator::SimpleTranslator, vms_core::RouteTranslator)
