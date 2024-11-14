import launch
from   launch.substitutions              import Command, LaunchConfiguration
from   launch.actions                    import IncludeLaunchDescription
from   launch.launch_description_sources import PythonLaunchDescriptionSource
from   ament_index_python.packages       import get_package_share_directory
import launch_ros
import os

def generate_launch_description():
    wheeltec_robot_pkg_share    = launch_ros.substitutions.FindPackageShare(package = 'turn_on_wheeltec_robot').find('turn_on_wheeltec_robot')
    my_bot_pkg_share   = launch_ros.substitutions.FindPackageShare(package = 'my_bot').find('my_bot')
    slam_toolbox_pkg_share = launch_ros.substitutions.FindPackageShare(package = 'slam_toolbox').find('slam_toolbox')
    model_path             = os.path.join(my_bot_pkg_share, 'description/robot.urdf.xacro')

    robot_state_publisher_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('my_bot'), 'launch', 'rsp.launch.py'
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
    turn_on_wheeltec_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('turn_on_wheeltec_robot'), 'launch'), '/turn_on_wheeltec_robot.launch.py'])
    )
    LDlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('turn_on_wheeltec_robot'), 'launch'), '/wheeltec_lidar.launch.py'])
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name = 'model'       , default_value = model_path, description = 'Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name = 'use_sim_time', default_value = 'False'   , description = 'Flag to enable use_sim_time'     ),

        robot_state_publisher_node,
        turn_on_wheeltec_robot_launch,  
        LDlidar_launch,
        slam_toolbox_node
    ])
