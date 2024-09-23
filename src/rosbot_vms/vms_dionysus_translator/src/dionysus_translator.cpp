#include "vms_dionysus_translator/dionysus_translator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vms_dionysus_translator
{

// Conversion constants
const double EARTH_RADIUS = 6378137.0; // Earth's radius in meters

// Member function to convert latitude and longitude to Cartesian coordinates
void DionysusTranslator::latLongToCartesian(double latitude, double longitude, double &x, double &y)
{
    // Convert latitude and longitude from degrees to radians
    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;

    // Simple equirectangular projection
    x = EARTH_RADIUS * lon_rad * cos(lat_rad);
    y = EARTH_RADIUS * lat_rad;
}

void DionysusTranslator::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & /*parent*/,
    std::string /*name*/)
{
    // Initialization code, if any
    RCLCPP_INFO(rclcpp::get_logger("DionysusTranslator"), "Configuring Dionysus Translator");
}

void DionysusTranslator::cleanup()
{
    // Cleanup resources, if any
    RCLCPP_INFO(rclcpp::get_logger("DionysusTranslator"), "Cleaning up Dionysus Translator");
}

void DionysusTranslator::activate()
{
    // Code to activate translator
    RCLCPP_INFO(rclcpp::get_logger("DionysusTranslator"), "Activating Dionysus Translator");
}

void DionysusTranslator::deactivate()
{
    // Code to deactivate translator
    RCLCPP_INFO(rclcpp::get_logger("DionysusTranslator"), "Deactivating Dionysus Translator");
}

nav_msgs::msg::Path DionysusTranslator::convertRoute(const vms_msgs::msg::Route & route)
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Time(); // Set to current time or appropriate value
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

} // namespace vms_dionysus_translator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vms_dionysus_translator::DionysusTranslator, vms_core::RouteTranslator)
