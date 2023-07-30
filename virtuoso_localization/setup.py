from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'virtuoso_localization'

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

        (os.path.join('share', package_name, 'config/vrx'),
            glob(os.path.join('config', 'vrx', '*.yaml'))),
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
            'republisher = virtuoso_localization.republisher_node:main',
            'test_pub = virtuoso_localization.testing.test_data_publisher:main',
            'localization_debugger = virtuoso_localization.localization_debugger_node:main',

            'keyboard_listener = virtuoso_localization.utils.keyboard_listener_node:main',
            'waypoint_saver = virtuoso_localization.waypoint_saver_node:main',
            'multi_tasks_waypoint_saver = virtuoso_localization.multi_tasks_waypoint_saver_node:main'
        ],
    },
)
