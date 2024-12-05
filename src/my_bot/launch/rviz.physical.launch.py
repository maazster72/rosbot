import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package name where the robot_state_publisher is located
    package_name = 'my_bot'  # Change this to your package name

    # RViz2 node to visualize the robot and laser scans
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'config.physical.rviz')],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        rviz
    ])

