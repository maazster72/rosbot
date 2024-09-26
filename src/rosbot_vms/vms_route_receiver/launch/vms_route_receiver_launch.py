#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vms_route_receiver',
            executable='route_receiver',
            name='vms_route_receiver_node',
            output='screen',
        )
    ])
