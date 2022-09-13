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
            'basic_pid = virtuoso_controller.basic_pid:main',   
            'velocity_pid = virtuoso_controller.velocity_pid:main', 
            'test_waypoint_generator = virtuoso_controller.testing.test_waypoint_generator:main',       
            'test_vel_generator = virtuoso_controller.testing.test_vel_generator:main',
            'test_forward = virtuoso_controller.testing.test_forward:main',
            'test_backward = virtuoso_controller.testing.test_backward:main',
            'test_left = virtuoso_controller.testing.test_left:main',
            'test_right = virtuoso_controller.testing.test_right:main',
            'motor_cmd_generator = virtuoso_controller.motor_cmd_generator:main',
        ],
    },
)
