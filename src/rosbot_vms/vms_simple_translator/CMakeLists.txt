cmake_minimum_required(VERSION 3.5)
project(vms_simple_translator)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vms_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vms_core REQUIRED)


nav2_package()

include_directories(
  include
)

set(library_name vms_simple_translator)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav_msgs
  geometry_msgs
  builtin_interfaces
  pluginlib
  vms_msgs
  vms_core
)

add_library(${library_name} SHARED
  src/simple_translator.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(vms_core simple_translator_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES simple_translator_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()