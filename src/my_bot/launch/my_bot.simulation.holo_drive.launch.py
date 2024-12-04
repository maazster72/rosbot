import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions       import TimerAction
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    my_bot_pkg_share         = launch_ros.substitutions.FindPackageShare(package = 'my_bot').find('my_bot')
    model_path               = os.path.join(my_bot_pkg_share, 'description/robot.urdf.xacro')
    slam_toolbox_pkg_share   = launch_ros.substitutions.FindPackageShare(package = 'slam_toolbox').find('slam_toolbox')

    world_path               = os.path.join(my_bot_pkg_share, 'worlds/5x5_world.sdf'),
    rviz_config_path         = os.path.join(my_bot_pkg_share, 'rviz/config.physical.rviz')

    launch_sim_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('my_bot'), 'launch', 'launch_sim.launch.py'
                )])
    )

    slam_toolbox_node = launch_ros.actions.Node(
        package    = 'slam_toolbox',
        executable = 'async_slam_toolbox_node',
        name       = 'slam_toolbox',
        output     = 'screen',
        parameters = [os.path.join(slam_toolbox_pkg_share, 'config', 'mapper_params_online_async.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    vms_guidance_interface_node = launch_ros.actions.Node(
        package    = 'vms_guidance_interface' ,
        executable = 'guidance_interface',
        name       = 'guidance_interface',
        output     = 'screen'
    )

    vms_controller_interface_node = launch_ros.actions.Node(
        package    = 'vms_controller_interface' ,
        executable = 'vms_controller_simulation_holo_drive',
        name       = 'vms_controller',
        output     = 'screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name = 'model'       , default_value = model_path      , description = 'Absolute path to robot urdf file' ),
        launch.actions.DeclareLaunchArgument(name = 'rvizconfig'  , default_value = rviz_config_path, description = 'Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name = 'use_sim_time', default_value = 'True'          , description = 'Flag to enable use_sim_time'      ),
        launch.actions.ExecuteProcess       (cmd  = ['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output = 'screen'           ),
        
        launch_sim_node,
        slam_toolbox_node,
        vms_guidance_interface_node,
        vms_controller_interface_node
    ])
