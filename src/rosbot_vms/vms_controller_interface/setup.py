from setuptools import setup
from glob import glob
import os

package_name = 'vms_controller_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maaz Ahmed',
    maintainer_email='mabdulahmed02@gmail.com',
    description='A package for controlling the robot using a planned path',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vms_controller_physical_holo_drive = vms_controller_interface.path_follower_physical_holo_drive:main',
            'vms_controller_simulation_holo_drive = vms_controller_interface.path_follower_simulation_holo_drive:main',
            'vms_controller_physical_directional_drive = vms_controller_interface.path_follower_physical_directional_drive:main',
            'vms_controller_simulation_directional_drive = vms_controller_interface.path_follower_simulation_directional_drive:main',
            'vms_nav2_controller = vms_controller_interface.vms_nav2_controller:main',
        ],
    },
)
