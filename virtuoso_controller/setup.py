from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'virtuoso_controller'

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
            'basic_pid = virtuoso_controller.basic_pid_node:main',   
            'velocity_pid = virtuoso_controller.velocity_pid_node:main', 
            'choose_pid = virtuoso_controller.choose_pid_node:main',
            'motor_cmd_generator = virtuoso_controller.motor_cmd_generator_node:main',
            'cmd_vel_generator = virtuoso_controller.cmd_vel_generator_node:main',

            'test_waypoint_generator = virtuoso_controller.testing.test_waypoint_generator:main',       
            'test_vel_generator = virtuoso_controller.testing.test_vel_generator:main',
            'test_stop = virtuoso_controller.testing.test_stop:main',
            'test_forward = virtuoso_controller.testing.test_forward:main',
            'test_backward = virtuoso_controller.testing.test_backward:main',
            'test_left = virtuoso_controller.testing.test_left:main',
            'test_right = virtuoso_controller.testing.test_right:main',
            'test_yaw_left = virtuoso_controller.testing.test_yaw_left:main',
            'test_yaw_right = virtuoso_controller.testing.test_yaw_right:main',
        ],
    },
)
