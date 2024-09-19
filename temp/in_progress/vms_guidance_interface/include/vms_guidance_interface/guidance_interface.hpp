#ifndef VMS_GUIDANCE_INTERFACE__GUIDANCE_INTERFACE_HPP_
#define VMS_GUIDANCE_INTERFACE__GUIDANCE_INTERFACE_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "vms_core/route_translator.hpp"
#include "vms_msgs/action/translate_route_to_path.hpp"
#include "vms_msgs/msg/route.hpp"

namespace vms_guidance_interface
{

	/**
	 * @class vms_guidance_interface::GuidanceInterface
	 * @brief An action server implements the TranslateRouteToPath interface and hosts plugins of different 
	 * translator algorithms to translate routes
	 */
class GuidanceInterface : public nav2_util::LifecycleNode

{
public:

	/**
	 * @function GuidanceInterface
	 * @brief A constructor for vms_guidance_interface::GuidanceInterface
	 * @param options Additional options to control the creation of the node.
	 */
	 explicit GuidanceInterface(
	 	const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
	 
	 /**
	  * @function ~GuidanceInterface
	  * @brief A destructor for vms_guidance_interface::GuidanceInterface
	  */
	 ~GuidanceInterface();
	 
	/**
	 * @var TranslatorMap
	 * @brief Associates a translator name (string) with a GlobalPlanner instance, managing various route
	 * translation plugins
	 */
	 using TranslatorMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;
	 
	/**
	 * @function getPath
	 * @brief Method to get path from the desired plugin
	 * @param route Provided route
	 * @param translator_id The translator to translate a route with
	 * @param cancel_checker A function to check if the action has been canceled
	 * @return Path
	 */
	 nav_msgs::msg::Path getPath(
	 	const vms_msgs::msg::Route & route,
	 	const std::string & translator_id,
	 	std::function<bool()> cancel_checker);
   	
protected:

	/**
	 * @function on_configure
	 * @brief Configure member variables and initialises guidance_interface
	 * @param state Reference to LifeCycle node state
	 * @return SUCCESS or FAILURE
	 */
	 nav2_util::CallbackReturn on_configure(
	 	const rclcpp_lifecycle::State & state) override;
	 
	/**
	 * @function on_activate
	 * @brief Activate member variables
	 * @param state Reference to LifeCycle node state
	 * @return SUCCESS or FAILURE
	 */
	 nav2_util::CallbackReturn on_activate(
	 	const rclcpp_lifecycle::State & state) override;
	 
	/**
	 * @function on_deactivate
	 * @brief Deactivate member variables
	 * @param state Reference to LifeCycle node state
	 * @return SUCCESS or FAILURE 
	 */
	 nav2_util::CallbackReturn on_deactivate(
	 	const rclcpp_lifecycle::State & state) override;
	 
	/**
	 * @function on_cleanup
	 * @brief Reset member variables
	 * @param state Reference to LifeCycle node state
	 * @return SUCCESS or FAILURE
	 */
	 nav2_util::CallbackReturn on_cleanup(
	 	const rclcpp_lifecycle::State & state) override;
	 
	/**
	 * @function on_shutdown
	 * @brief Called when in shutdown state
	 * @param state Reference to LifeCycle node state
	 * @return SUCCESS or FAILURE
	 */
	 nav2_util::CallbackReturn on_shutdown(
	 	const rclcpp_lifecycle::State & state) override;
	 
	/**
	 * @var ActionToPath
	 * @brief Alias for the action type related to TranslateRouteToPath.
	 */
	using ActionToPath = vms_msgs::action::TranslateRouteToPath;

	/**
	 * @var ActionToPathResult
	 * @brief Alias for the result type of the TranslateRouteToPath action.
	 */
	using ActionToPathResult = ActionToPath::Result;

	/**
	 * @var ActionServerToPath
	 * @brief Alias for the SimpleActionServer handling the TranslateRouteToPath action.
	 */
	using ActionServerToPath = nav2_util::SimpleActionServer<ActionToPath>;

	/**
	 * @function isServerInactive
	 * @brief Check if an action server is valid / active
	 * @param action_server Action server to test
	 * @return SUCCESS or FAILURE
	 */
	 template<typename T>
	 bool isServerInactive(
	 	std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);
	 
	/**
	* @function isCancelRequested
	* @brief Check if an action server has a cancellation request pending
	* @param action_server Action server to test
	* @return SUCCESS or FAILURE
	*/
	template<typename T>
	bool isCancelRequested(
		std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

	/**
	* @function getPreemptedRouteIfRequested
	* @brief Check if an action server has a preemption request and replaces the route
	* with the new preemption goal.
	* @param action_server Action server to get updated route if required
	* @param route Route to overwrite
	*/
	template<typename T>
	void getPreemptedRouteIfRequested(
		std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
		typename std::shared_ptr<const typename T::Route> route);

	/**
	* @function validatePath
	* @brief Validate that the path contains a meaningful path
	* @param route Current Route
	* @param path Current path
	* @param translator_id The translator ID used to translate the route
	* @return bool If path is valid
	*/
	template<typename T>
	bool ValidatePath(
		const vms_msgs::msg::Route & curr_route,
		const nav_msgs::msg::Path & path,
		const std::string & translator_id);
	  	
	/**
	* @function translateRoute
	* @brief The action server callback which calls the translator to translate the route to a path
	*/
	void translateRoute();
	   
	/**
	* @function isPathValid
	* @brief The service callback to determine if the path is still valid
	* @param request to the service
	* @param response to the service
	*/
	void isPathValid(
	const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
	std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);
	   	
	/**
	 * @function publishPath
	 * @brief Publish a path for visualisation purposes or further processing
	 * @param path Reference to Global Path
	 */
	 void publishPath(
	 	const nav_msgs::msg::Path & path);
	 	
	/**
	 * @function exceptionWarning
	 * @brief Logs a warning message when an exception occurs during translation
	 * @param route The provided route to translate
	 * @param translator_id The translator ID used to translate the route
	 * @param ex The exception that was thrown.
	 */
	 void exceptionWarning(
	 	const vms_msgs::msg::Route & route,
	 	const std::string & translator_id,
	 	const std::exception & ex);
	 	
	/**
	 * @function dynamicParametersCallback
	 * @brief Callback executed when a paramter change is detected
	 */
	 void dynamicParametersCallback(
	 	std::vector<rclcpp::Parameter> parameters);
	 	
	// Our action server implements the TranslateRouteToPath action
	std::unique_ptr<ActionServerToPath> action_server_path_;
	
	// Dynamic parameters handler
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
	std::mutex dynamic_params_lock_;
	
	// Translator
	TranslatorMap translators_;
	pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
	std::vector<std::string> default_ids_;
	std::vector<std::string> default_types_;
	std::vector<std::string> translator_ids_;
	std::vector<std::string> translator_types_;
	double max_translator_duration_;
	std::string translator_ids_concat_;
	
	// Publishers for the path
	rclcpp_lifecycle::LifeCyclePublisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
	
	// Service to determine if the path is still valid
	rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr is_path_valid_service_;
};

} // namespace vms_guidance_interface

#endif // VMS_GUIDANCE_INTERFACE__GUIDANCE_INTERFACE_HPP__

