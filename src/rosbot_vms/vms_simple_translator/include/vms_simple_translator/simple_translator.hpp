#ifndef VMS_SIMPLE_TRANSLATOR__SIMPLE_TRANSLATOR_HPP_
#define VMS_SIMPLE_TRANSLATOR__SIMPLE_TRANSLATOR_HPP_

#include <memory>
#include <string>
#include "vms_core/route_translator.hpp"
#include "vms_msgs/msg/route.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vms_simple_translator
{

class SimpleTranslator : public vms_core::RouteTranslator
{
public:
    using Ptr = std::shared_ptr<SimpleTranslator>;

    SimpleTranslator() = default;
    ~SimpleTranslator() override = default;

    // Member function to convert latitude and longitude to Cartesian coordinates
    void latLongToCartesian(double latitude, double longitude, double &x, double &y);

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path convertRoute(const vms_msgs::msg::Route & route) override;

private:
    rclcpp::Clock::SharedPtr clock_;  // Declare the clock_ member variable
};

} // namespace vms_simple_translator

#endif // VMS_SIMPLE_TRANSLATOR__SIMPLE_TRANSLATOR_HPP_
