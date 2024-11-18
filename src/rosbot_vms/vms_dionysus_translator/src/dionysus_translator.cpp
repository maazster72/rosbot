#include "vms_dionysus_translator/dionysus_translator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

namespace vms_dionysus_translator
{

// Conversion constants
const double EARTH_RADIUS = 6378137.0; // Earth's radius in meters

// Member function to convert latitude and longitude to Cartesian coordinates
void DionysusTranslator::latLongToCartesian(double latitude, double longitude, double &x, double &y)
{
    // Define initial reference point (latitude, longitude)
    double initial_latitude = 53.745793304041634;
    double initial_longitude = -2.894669081029672;
    // Scale factor to control the precision of the conversion
    double scale_factor = 10000 * 1.5;

    // Simple equirectangular projection
    x = (latitude - initial_latitude) * scale_factor * -1;
    y = (longitude - initial_longitude) * scale_factor;
}

void DionysusTranslator::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & /*parent*/,
    std::string /*name*/)
{
    // Initialize the clock
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
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
    path.header.stamp = clock_->now(); // Set to current time using the clock_
    path.header.frame_id = "map"; // Set to the appropriate frame

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
