from setuptools import setup
from glob import glob
import os

package_name = 'vms_route_receiver'

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
    description='A package to retreive route from an MQTT broken and call the translateRouteToPath action',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_receiver = vms_route_receiver.route_receiver:main',
        ],
    },
)
