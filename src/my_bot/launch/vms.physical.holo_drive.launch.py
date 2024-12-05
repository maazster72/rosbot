import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions       import TimerAction
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    vms_guidance_interface_node = launch_ros.actions.Node(
        package    = 'vms_guidance_interface' ,
        executable = 'guidance_interface',
        name       = 'guidance_interface',
        output     = 'screen'
    )

    vms_controller_interface_node = launch_ros.actions.Node(
        package    = 'vms_controller_interface' ,
        executable = 'vms_controller_physical_holo_drive',
        name       = 'vms_controller',
        output     = 'screen'
    )

    return launch.LaunchDescription([
        vms_guidance_interface_node,
        vms_controller_interface_node
    ])
