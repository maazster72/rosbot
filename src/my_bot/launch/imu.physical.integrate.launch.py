import launch
from   launch.substitutions              import Command, LaunchConfiguration
from   launch.actions                    import IncludeLaunchDescription
from   launch.launch_description_sources import PythonLaunchDescriptionSource
from   ament_index_python.packages       import get_package_share_directory
import launch_ros
import os

def generate_launch_description():
    my_bot_pkg_share = launch_ros.substitutions.FindPackageShare(package = 'my_bot').find('my_bot')
    model_path          = os.path.join(my_bot_pkg_share, 'description/my_bot.physical.fixed.wheels.urdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    imuBroadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('my_bot'), 'launch'), '/imu_broadcaster_launch.py'])
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name = 'model'       , default_value = model_path, description = 'Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name = 'use_sim_time', default_value = 'False'   , description = 'Flag to enable use_sim_time'     ),
        imuBroadcaster_launch
    ])
