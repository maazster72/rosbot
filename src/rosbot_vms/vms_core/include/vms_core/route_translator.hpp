#ifndef VMS_CORE__ROUTE_TRANSLATOR_HPP_
#define VMS_CORE__ROUTE_TRANSLATOR_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include  "vms_msgs/msg/route.hpp"

namespace vms_core
{

/*
 * @class RouteTranslator
 * @brief Abstract interface for route translators to adhere to with plugin lib
 */
class RouteTranslator
{
public:
    using Ptr = std::shared_ptr<RouteTranslator>;

    /*
     * @brief Virtual destructor
     */
    virtual ~RouteTranslator() {}

    /*
     * @param parent pointer to user's node
     * @param name The name of this translator
     */
    virtual void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name) = 0;

    /*
     * @brief Method to cleanup resources used on shutdown
     */
    virtual void cleanup() = 0;

    /*
     * @brief Method to activate translator and any threads involved in execution
     */
    virtual void activate() = 0;

    /*
     * @brief Method to deactivate translator and any threads involved in execution
     */
    virtual void deactivate() = 0;


    /*
     * @brief Method to translate a route to a navigation path
     * @param route The route to translate
     * @param cancel_checker A callable function to check if the action has been canceled, which should return true if the action needs to be stopped.
     * @return The translated path derived from the route which is a sequence of poses to get from start to goal
     */
    virtual nav_msgs::msg::Path convertRoute(
        const vms_msgs::msg::Route & route) = 0;
};


} // namespace vms_core

#endif // VMS_CORE__ROUTE_TRANSLATOR_HPP_