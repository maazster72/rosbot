#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vms_controller_interface',
            executable='vms_pure_pursuit',
            name='vms_pure_pursuit_node',
            output='screen',
        )
    ])
