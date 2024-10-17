import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions       import TimerAction
import launch_ros
import os

def generate_launch_description():
    my_bot_pkg_share         = launch_ros.substitutions.FindPackageShare(package = 'my_bot').find('my_bot')
    model_path               = os.path.join(my_bot_pkg_share, 'description/robot_car.vm.simulation.urdf')
    world_path               = os.path.join(my_bot_pkg_share, 'worlds/my_world.sdf'),

    robot_state_publisher_node = launch_ros.actions.Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    spawn_gazebo_entity = launch_ros.actions.Node(
        package    = 'gazebo_ros',
        executable =  'spawn_entity.py',
        arguments  = ['-entity', 'my_bot', '-topic', 'robot_description'],
        output     = 'screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name = 'model'       , default_value = model_path      , description = 'Absolute path to robot urdf file' ),
        launch.actions.DeclareLaunchArgument(name = 'use_sim_time', default_value = 'True'          , description = 'Flag to enable use_sim_time'      ),
        launch.actions.ExecuteProcess       (cmd  = ['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output = 'screen'           ),
        
        robot_state_publisher_node,
        spawn_gazebo_entity
    ])
