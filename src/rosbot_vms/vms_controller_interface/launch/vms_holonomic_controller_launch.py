def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vms_controller_interface',
            executable='vms_controller_directional_drive',
            name='vms_controller_node',
            output='screen',
        )
    ])