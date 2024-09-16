cmake_minimum_required(VERSION 3.5)
project(vms_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

set(MSG_FILES
	"msg/RoutePoint.msg"
	"msg/Route.msg"
)

set(ACTION_FILES
	"action/TranslateRouteToPath.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${MSG_FILES}
	${ACTION_FILES}
	DEPENDENCIES builtin_interfaces nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY msg action
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
