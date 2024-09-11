import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package name where the robot_state_publisher is located
    package_name = 'my_bot'  # Change this to your package name

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
                )]), 
                launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
             )

    # Spawner for the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen',
        remappings=[('~/out', 'scan')]  # Remap laser scan topic to 'scan'
    )

    # RViz2 node to visualize the robot and laser scans
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'drive_bot.rviz')],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz,
    ])

