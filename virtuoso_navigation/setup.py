from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'virtuoso_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),

        (os.path.join('share', package_name, 'config/vrx_robotx'), 
            glob(os.path.join('config', 'vrx_robotx', '*.yaml'))),

        (os.path.join('share', package_name, 'config/robotx'),
            glob(os.path.join('config', 'robotx', '*.yaml'))),

        (os.path.join('share', package_name, 'config/vrx_roboboat'),
            glob(os.path.join('config', 'vrx_roboboat', '*.yaml'))),

        (os.path.join('share', package_name, 'config/roboboat'),
            glob(os.path.join('config', 'roboboat', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gtmrg',
    maintainer_email='gtmrg@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoints = virtuoso_navigation.waypoints_node:main',
            'translate = virtuoso_navigation.translate_node:main',
            'station_keeping = virtuoso_navigation.station_keeping_node:main',
            'rotate = virtuoso_navigation.rotate_node:main',

            'waypoint_player = virtuoso_navigation.waypoint_player_node:main',
            'multi_tasks_waypoint_player = virtuoso_navigation.multi_tasks_waypoint_player_node:main',

            'test_set_goal = virtuoso_navigation.testing.set_goal:main',
            'test_change_goal = virtuoso_navigation.testing.change_goal:main',
            'test_waypoints = virtuoso_navigation.testing.waypoints:main',
            'test_controller = virtuoso_navigation.testing.controller:main',
            'test_forward = virtuoso_navigation.testing.forward:main',
            'test_backward = virtuoso_navigation.testing.backward:main',
            'test_translate = virtuoso_navigation.testing.translate_node:main',
            'test_diamond = virtuoso_navigation.testing.diamond:main',
            'test_station_keeping = virtuoso_navigation.testing.station_keeping:main',
            'test_circle = virtuoso_navigation.testing.circle:main'
        ],
    },
)
