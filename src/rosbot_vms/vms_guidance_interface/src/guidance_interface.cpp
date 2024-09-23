#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/node_utils.hpp"

#include "vms_guidance_interface/guidance_interface.hpp"

using namespace std::chrono_literals;

namespace vms_guidance_interface
{

GuidanceInterface::GuidanceInterface(const rclcpp::NodeOptions & /*options*/)
: nav2_util::LifecycleNode("vms_guidance_interface", "", true),
translator_loader_("vms_core", "vms_core::RouteTranslator"),
default_ids_{"GridBased"},
default_types_{"vms_dionysus_translator/DionysusTranslator"}
{
    RCLCPP_INFO(
        get_logger(), "Creating");

    // Declare this node's parameters
    declare_parameter("translator_plugins", default_ids_);
    declare_parameter("expected_translator_frequency", 1.0);

    // Declare parameters for default plugins if none are set
    get_parameter("translator_plugins", translator_ids_);
    if(translator_ids_ == default_ids_) {
        for (size_t i = 0; i < default_ids_.size(); ++i) {
            declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
        }
    }

}
	 
GuidanceInterface::~GuidanceInterface()
{
  /*
    * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
    * never called 
    */
  translators_.clear();
}

nav2_util::CallbackReturn GuidanceInterface::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring Guidance Interface: Initialising translator plugins, action servers, and publishers.");

    translator_types_.resize(translator_ids_.size());

    auto node = shared_from_this();

    for (size_t i = 0; i != translator_ids_.size(); i++) {
        try {
            translator_types_[i] = nav2_util::get_plugin_type_param(
                node, translator_ids_[i]);
            vms_core::RouteTranslator::Ptr translator =
                translator_loader_.createUniqueInstance(translator_types_[i]);
            RCLCPP_INFO(
                get_logger(), "Created translator plugin %s of type %s",
                translator_ids_[i].c_str(), translator_types_[i].c_str());
            translator->configure(node, translator_ids_[i]);
            translators_.insert({translator_ids_[i], translator});
            } catch (const std::exception & ex) {
            RCLCPP_FATAL(
                get_logger(), "Failed to create translator. Exception: %s",
                ex.what());
            return nav2_util::CallbackReturn::FAILURE;
        }
    }

    for (size_t i = 0; i != translator_ids_.size(); i++) {
        translator_ids_concat_ += translator_ids_[i] + std::string(" ");
    }

    RCLCPP_INFO(
        get_logger(),
        "Guidance Interface has %s translators availble.", translator_ids_concat_.c_str()
    );

    double expected_translator_frequency;
    get_parameter("expected_translator_frequency", expected_translator_frequency);
    if (expected_translator_frequency > 0) {
        max_translator_duration_ = 1 / expected_translator_frequency;
    } else {
        RCLCPP_WARN(
            get_logger(),
            "The expected planner frequency parameter is %.4f Hz. The value should be greater"
            " than 0.0 to turn on duration overrun warning messages", expected_translator_frequency);
        max_translator_duration_ = 0.0;
    }

    // Initialise pubs & subs
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

    // Create the action servers for translating a route to a path
    action_server_path_ = std::make_unique<ActionServerTranslateRouteToPath>(
        rclcpp_node_,
        "translate_route_to_path",
        std::bind(&GuidanceInterface::translateRoute, this));

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidanceInterface::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), 
    "Activating Guidance Interface: Publishers, action servers, translators, and services initialised.");


    path_publisher_->on_activate();
    action_server_path_->activate();

    TranslatorMap::iterator it;
    for (it = translators_.begin(); it != translators_.end(); ++it) {
        it->second->activate();
    }

    auto node = shared_from_this();

    // create bond connection
    createBond();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidanceInterface::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), 
    "Deactivating Guidance Interface: Action server, publisher, and translators are now inactive.");

    action_server_path_->deactivate();
    path_publisher_->on_deactivate();

    TranslatorMap::iterator it;
    for (it = translators_.begin(); it != translators_.end(); ++it) {
        it->second->deactivate();
    }

    // Destroy bond connection
    destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidanceInterface::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Cleaning up Guidance Interface: Resetting action server, publisher, and translators");

    action_server_path_.reset();
    path_publisher_.reset();

    TranslatorMap::iterator it;
    for (it = translators_.begin(); it != translators_.end(); ++it) {
        it->second->cleanup();
    }

    translators_.clear();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidanceInterface::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Shutting down Guidance Interface and releasing resources");
    return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool GuidanceInterface::isServerInactive(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
    if (action_server == nullptr || !action_server->is_server_active()) {
        RCLCPP_WARN(get_logger(), "Action Server unavilable or inactive. Stopping.");
        return true;
    }

    return false;
}

template<typename T>
bool GuidanceInterface::isCancelRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
    if (action_server->is_cancel_requested()) {
        RCLCPP_WARN(get_logger(), "Goal was cancelled. Cancelling translation action.");
        action_server->terminate_all();
        return true;
}

return false;
}

template<typename T>
void GuidanceInterface::getPreemptedGoalIfRequested(
std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
typename std::shared_ptr<const typename T::Goal> goal)
{
    if (action_server->is_preempt_requested()) {
        goal = action_server->accept_pending_goal();
    }
}

template<typename T>
bool GuidanceInterface::validatePath(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    const vms_msgs::msg::Route & /*curr_route*/,
    const nav_msgs::msg::Path & path,
    const std::string & translator_id)
{
    if (path.poses.size() == 0) {
        RCLCPP_WARN(
            get_logger(), "Translation algorithm %s failed to translate route.", translator_id.c_str());
        action_server->terminate_current();
        return false;
    }

    RCLCPP_DEBUG(
        get_logger(),
        "Translated to valid path of size %zu", path.poses.size()
    );

    return true;
}

void GuidanceInterface::translateRoute()
{
    auto start_time = steady_clock_.now();

    // Initialise the TranslateRouteToPath goal and result
    auto goal = action_server_path_->get_current_goal();
    auto result = std::make_shared<ActionTranslateRouteToPath::Result>();

    vms_msgs::msg::Route route = goal->route;
    
    try {
        if (isServerInactive(action_server_path_) || isCancelRequested(action_server_path_)) {
            return;
        }

        getPreemptedGoalIfRequested(action_server_path_, goal);

        result->path = getPath(route, goal->translator_id);

        if (!validatePath(action_server_path_, route, result->path, goal->translator_id)) {
            throw vms_core::NoValidPathCouldBeFound(goal->translator_id + " generated a empty path");
        }

        publishPath(result->path);

        auto cycle_duration = steady_clock_.now() - start_time;
        result->translation_time = cycle_duration;

        if (max_translator_duration_ && cycle_duration.seconds() > max_translator_duration_) {
            RCLCPP_WARN(
                get_logger(),
                "Translator loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
                1 / max_translator_duration_, 1 / cycle_duration.seconds());
        }

        action_server_path_->succeeded_current(result);
    } catch (std::exception & ex) {
        RCLCPP_WARN(
            get_logger(), "%s plugin failed to translate the route: \"%s\"",
            goal->translator_id.c_str(), ex.what());
    }
}
	 
nav_msgs::msg::Path GuidanceInterface::getPath(
    const vms_msgs::msg::Route & route,
    const std::string & translator_id)
{
    RCLCPP_DEBUG(get_logger(), "Attempting to translate route to path...");

    if (translators_.find(translator_id) != translators_.end()) {
        return translators_[translator_id]->convertRoute(route);
    } else {
        if (translators_.size() == 1 && translator_id.empty()) {
            RCLCPP_WARN_ONCE(
                get_logger(), "No translators specified in action call. "
                "Guidance Interface will use only plugin %s in server."
                "This warning will appear one.", translator_ids_concat_.c_str());
            return translators_[translators_.begin()->first]->convertRoute(route);
        } else {
            RCLCPP_ERROR(
                get_logger(), "translator %s is not a valid translator. "
                "Translator names are: %s", translator_id.c_str(), translator_ids_concat_.c_str()
            );
            throw vms_core::InvalidTranslator("Translator id " + translator_id + "is invalid");
        }
    }

    return nav_msgs::msg::Path();
}

void GuidanceInterface::publishPath(const nav_msgs::msg::Path & path)
{
    auto msg = std::make_unique<nav_msgs::msg::Path>(path);
    if (path_publisher_->is_activated() && path_publisher_->get_subscription_count() > 0) {
        path_publisher_->publish(std::move(msg));
    }
}

void GuidanceInterface::exceptionWarning(
    const vms_msgs::msg::Route & /*route*/,
    const std::string & translator_id,
    const std::exception & ex)
{
    RCLCPP_WARN(
    get_logger(), "%s plugin failed to translate route \"%s\"",
    translator_id.c_str(),
    ex.what());
}
    
} // namespace vms_guidance_interface

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vms_guidance_interface::GuidanceInterface)

