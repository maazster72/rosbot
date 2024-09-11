# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.5)

# Define the project name
project(my_global_planner)

# Set default C standard to C99
set(CMAKE_C_STANDARD 99)

# Set default C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)

# Find necessary packages and dependencies
find_package(ament_cmake REQUIRED) # CMake utilities for ROS 2 packages
find_package(rclcpp REQUIRED) # ROS 2 C++ client library
find_package(rclcpp_action REQUIRED) # Action client and server for ROS 2
find_package(rclcpp_lifecycle REQUIRED) # Lifecycle management for nodes
find_package(std_msgs REQUIRED) # Standard messages for ROS 2
find_package(visualization_msgs REQUIRED) # Messages for visualization in ROS 2
find_package(nav2_util REQUIRED) # Navigation utility functions
find_package(nav2_msgs REQUIRED) # Navigation messages for ROS 2
find_package(nav_msgs REQUIRED) # Navigation messages
find_package(geometry_msgs REQUIRED) # Geometry messages
find_package(builtin_interfaces REQUIRED) # Built-in interfaces for ROS 2
find_package(tf2_ros REQUIRED) # ROS 2 TF2 library for transforming data
find_package(nav2_costmap_2d REQUIRED) # Costmap functionalities for navigation
find_package(nav2_core REQUIRED) # Core functionalities for navigation plugins
find_package(pluginlib REQUIRED) # Library for dynamic loading of plugins

# Include directories for header files
include_directories(
  include
)

# Define the name of the shared library
set(library_name ${PROJECT_NAME}_plugin)

# List of dependencies for the library
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
)

# Create the shared library for the global planner
add_library(${library_name} SHARED
  src/my_global_planner.cpp # Source file for the global planner
)

# Link the dependencies to the library
ament_target_dependencies(${library_name}
  ${dependencies}
)

# Define compile definitions for the plugin
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Export the plugin description file for pluginlib
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

# Install directives for the library
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib # Install static library to lib
  LIBRARY DESTINATION lib # Install shared library to lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Install runtime files
)

# Install the header files from the include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install the plugin description XML file
install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Conditional testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED) # Find linting tools for ROS 2
  # Skip linter checks if copyrights are missing
  # Uncomment the next line when a copyright/license is present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint if not in a git repository
  # Uncomment the next line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies() # Automatically find test dependencies
endif()

# Export include directories and libraries for use in other packages
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

# Mark the package as an ament package
ament_package()

