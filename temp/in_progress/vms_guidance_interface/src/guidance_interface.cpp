#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <chrono>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"

#include "vms_msgs/action/translate_route_to_path.hpp"
#include "vms_msgs/msg/route.hpp"

#include "vms_guidance_interface/guidance_interface.hpp"

using namespace std::chrono_literals
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;


namespace vms_guidance_interface
{

    GuidanceInterface::GuidanceInterface(const rclcpp::NodeOptions & options)
    : nav2_util::LifecycleNode("guidance_interface", "", options),
    gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
    default_ids_{"GridBased"},
    default_types_{"vms_dionysus_translator::DionysusTranslator"}
    {
        RCLCPP_INFO(
            get_logger(), 
            "Creating GuidanceInterface Node. Loading translator plugins: [%s]. Action server result timeout: %.2f seconds.",
            rclcpp::join(translator_ids_, ", ").c_str(),
            get_parameter("action_server_result_timeout").as_double()
        );

        // Declare this node's parameters
        declare_parameter("translator_plugins", default_ids_);
        declare_parameter("action_server_result_timeout", 10.0);

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
            nav2_core::GlobalPlanner::Ptr translator =
                gp_loader_.createUniqueInstance(translator_types_[i]);
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

        // Initialise pubs & subs
        path_publisher_ = create_publisher<nav_msgs::msg::Path>("path", 1);

        double action_server_result_timeout;
        get_parameter("action_server_result_timeout", action_server_result_timeout);
        rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
        server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

        // Create the action servers for translating a route to a path
        action_server_path_ = std::make_unique<ActionServerToPath>(
            shared_from_this(),
            "translate_route_to_path",
            std::bind(&GuidanceInterface::translateRoute, this),
            nullptr,
            std::chrono::milliseconds(500),
            true, server_options);

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

        is_path_valid_service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
            "is_path_valid",
            std::bind(
            &GuidanceInterface::isPathValid, this,
            std::placeholders::_1, std::placeholders::_2));

        // Add callback for dynamic parameters
        dyn_params_handler_ = node->add_on_set_parameters_callback(
            std::bind(&GuidanceInterface::dynamicParametersCallback, this, _1));

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
        for (it = translators_.begin(); it != translators_.end(); ++i) {
            it->second->deactivate();
        }

        dynamic_params_handler_.reset();

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
        for (it = translators_.begin(); it != translators_.end(); ++i) {
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
            RCLCPP_WARN(get_logger(), 
            "Action server '%s' is either not initialised or inactive. Stopping the process.", 
            action_server ? action_server->get_name().c_str() : "Unknown");
            return true;
        }
        
        return false;
    }

    template<typename T>
	 bool GuidanceInterface::isCancelRequested(
        std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
    {
        if (action_server->is_cancel_requested()) {
            RCLCPP_WARN(get_logger(), 
                "Cancellation request received for action server '%s'. Terminating all ongoing translations.", 
                action_server->get_name().c_str());
            action_server->terminate_all();
            return true;
        }
        
        return false;
    }

    template<typename T>
	void GuidanceInterface::getPreemptedRouteIfRequested(
        std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
        typename std::shared_ptr<const typename T::Route> route)
    {
        if (action_server->is_preempt_requested()) {
            route = action_server->accept_pending_goal();
        }
    }

    template<typename T>
	 bool GuidanceInterface::validatePath(
        const vms_msgs::msg::Route & curr_route,
        const nav_msgs::msg::Path & path,
        const std::string & translator_id)
    {
        if (path.poses.empty()) {
            RCLCPP_WARN(
                // TODO: Make message more descriptive by providing curr_route.c_str()
                get_logger(), "Translation algorithm %s failed to translate route.", translator_id);
            return false;
        }

        RCLCPP_DEBUG(
            get_logger(),
            "Translated to valid path of size %zu", path.poses.size()
        );

        return true;
    }

    void GuidanceInterface::translatePath()
    {
        std::lock_guard<std::mutex> lock(dynamic_params_lock_);

        auto start_time = this->now();

        // Initialise the TranslateRouteToPath route and result
        auto route = action_server_path_->get_current_route();
        auto result = std::make_shared<ActionToPath::Result>();

        vms_msgs::msg::Route route;

        try {
            if (isServerInactive(action_server_path_) || isCancelRequested(action_server_path_)) {
                return;
            }
            getPremptedRouteIfRequested(action_server_path_, route);

            auto cancel_checker = [this]() {
                return action_server_path_->is_cancel_requested();
            }

            result->path = getPath(route, route->translator_id, cancel_checker);

            if (!validatePath<ActionToPath>(route, result->path, route->translator_id)) {
                throw nav2_core::NoValidPathCouldBeFound(route->translator_id + " generated a empty path");
            }

            publishPath(result->path);

            auto cycle_duration = this->now() - start_time;
            result->planning_time = cycle_duration;

            RCLCPP_DEBUG(get_logger(), "Translator cycle duration was %s", cycle_duration.c_str());

            action_server_path_->succeeded_current(result);
        } catch (nav2_core::InvalidPlanner & ex) {
            exceptionWarning(route->, route->translator_id, ex)
            result->error_code = ActionToPoseResult::INVALID_PLANNER;
            action_server_path_->terminate_current(result);
        } // TODO: All exception cases
    }
	 
	nav_msgs::msg::Path GuidanceInterface::getPath(
	    const vms_msgs::msg::Route & route,
	 	const std::string & translator_id,
	 	std::function<bool()> cancel_checker)
    {
        RCLCPP_DEBUG(get_logger(), "Attempting to translate route to path...")

        if (translators_.find(translator_id) != translators_.end()) {
            return translators_[translator_id]->createPlan(route, cancel_checker);
        } else {
            if (translators_.size() == 1 && translator_id.empty()) {
                RCLCPP_WARN_ONCE(
                    get_logger(), "No translators specified in action call. "
                    "Guidance Interface will use only plugin %s in server."
                    "This warning will appear one.", translator_ids_concat_.c_str();
                );
                return translators_[translators_.begin()->first]->createPlan(route, cancel_checker);
            } else {
                RCLCPP_ERROR(
                    get_logger(), "translator %s is not a valid translator. "
                    "Translator names are: %s", translator_id.c_str(), translator_ids_concat_.c_str()
                );
                throw nav2_core::InvalidPlanner("Translator id " + translator_id + "is invalid");
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

    void GuidanceInterface::isPathValid(
        const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
	    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
    {
        // TODO: Implement function
        return;
    }

    rcl_interfaces::msg::SetParametersResult GuidanceInterface::dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters)
    {
        std::lock_guard<std::mutex> lock(dynamic_params_lock_);
        rcl_interfaces::msg::SetParametersResult result;
        for (auto parameter : parameters) {
            const auto & type = parameter.get_type();
            const auto & name = parameter.get_name();

            // if (type == ParameterType::PARAMETER_DOUBLE) {
            // if (name == "expected_planner_frequency") {
            //     if (parameter.as_double() > 0) {
            //     max_planner_duration_ = 1 / parameter.as_double();
            //     } else {
            //     RCLCPP_WARN(
            //         get_logger(),
            //         "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
            //         " than 0.0 to turn on duration overrrun warning messages", parameter.as_double());
            //     max_planner_duration_ = 0.0;
            //     }
            // }
        }

        result.successful = true;
        return result;
    }

    void GuidanceInterface::exceptionWarning(
	 	const vms_msgs::msg::Route & route,
	 	const std::string & translator_id,
	 	const std::exception & ex)
    {
        RCLCPP_WARN(
        get_logger(), "%s plugin failed to translate route \"$s\"",
        planner_id.c_str(),
        ex.what());
    }
    
} // namespace vms_guidance_interface

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vms_guidance_interface::GuidanceInterface)

