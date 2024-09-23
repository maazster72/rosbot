#ifndef VMS_DIONYSUS_TRANSLATOR__DIONYSUS_TRANSLATOR_HPP_
#define VMS_DIONYSUS_TRANSLATOR__DIONYSUS_TRANSLATOR_HPP_

#include <memory>
#include <string>
#include "vms_core/route_translator.hpp"
#include "vms_msgs/msg/route.hpp"
#include "nav_msgs/msg/path.hpp"

namespace vms_dionysus_translator
{

class DionysusTranslator : public vms_core::RouteTranslator
{
public:
    using Ptr = std::shared_ptr<DionysusTranslator>;

    DionysusTranslator() = default;
    ~DionysusTranslator() override = default;

    // Member function to convert latitude and longitude to Cartesian coordinates
    void latLongToCartesian(double latitude, double longitude, double &x, double &y);

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path convertRoute(const vms_msgs::msg::Route & route) override;
};

} // namespace vms_dionysus_translator

#endif // VMS_DIONYSUS_TRANSLATOR__DIONYSUS_TRANSLATOR_HPP_
