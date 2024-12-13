#include "vms_dionysus_translator/dionysus_translator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "geometry/Coordinates.hpp"
#include "geometry/Ellipsoid.hpp"


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

    // Define the origin (Dionysus reference point)
    geometry::Point origin(53.745620171847804, -2.8941631520855164, 0.0);

    for (const auto & point : route.routepoints) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;

        // Convert latitude/longitude to Cartesian coordinates
        geometry::Point geodeticPoint(point.latitude, point.longitude, 0.0);
        geometry::Cartesian cartesian =
            geometry::Coordinates::geodeticToCartesian(geometry::Ellipsoid::WGS84, geodeticPoint, origin);

        // Populate the ROS PoseStamped
        pose.pose.position.x = cartesian.e * -1;
        pose.pose.position.y = cartesian.n;
        pose.pose.position.z = 0.0;

        // Set orientation if needed (default to no rotation)
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
    }

    return path;
}

} // namespace vms_dionysus_translator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vms_dionysus_translator::DionysusTranslator, vms_core::RouteTranslator)
