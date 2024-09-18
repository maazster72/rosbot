cmake_minimum_required(VERSION 3.5)
project(vms_guidance_interface)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(vms_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(executable_name guidance_interface)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
	src/guidance_interface.cpp
)
target_include_directories(${library_name}
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
	${geometry_msgs_TARGETS}
	nav2_core::nav2_core
	${nav2_msgs_TARGETS}
	nav2_util::nav2_util_core
	${nav_msgs_TARGETS}
	pluginlib::pluginlib
	rclcpp::rclcpp
	rclcpp_lifecycle::rclcpp_lifecycle
	${rcl_interface_TARGETS}
	tf2_ros::tf2_ros
	${vms_msgs_LIBRARIES}
)
target_link_libraries(${library_name} PRIVATE
	${lifecycle_msgs_TARGETS}
	rclcpp_components::component
	tf2::tf2
)

add_executable(${executable_name}
	src/main.cpp
)
target_link_libraries(${executable_name} PRIVATE ${library_name} rclcpp::rclcpp)

rclcpp_components_register_nodes(${library_name} "vms_guidance_interface::GuidanceInterface")

install(TARGETS ${library_name}
	EXPORT ${library_name}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
	RUNTIME DESTINATION lib/${PROJECT_NAME}	
)

install(DIRECTORY include/
	DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_link_auto REQUIRED)
	find_package(ament_cmake_gtest REQUIRED)
	ament_lint_auto_find_test_dependencies()
	
	ament_find_gtest()
	add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
	geometry_msgs
	nav2_core
	nav2_msgs
	nav2_util
	nav_msgs
	pluginlib
	rclcpp
	rclcpp_lifecycle
	tf2_ros
	vms_msgs	
)
ament_export_targets(${library_name})
ament_package()
