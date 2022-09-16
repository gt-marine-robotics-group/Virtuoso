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
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.param.yaml')))
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
            'set_goal = virtuoso_navigation.set_goal:main',
            'waypoints = virtuoso_navigation.waypoints:main',
            'choose_PID = virtuoso_navigation.choose_PID:main',

            'test_set_goal = virtuoso_navigation.testing.set_goal:main',
            'test_change_goal = virtuoso_navigation.testing.change_goal:main',
            'test_waypoints = virtuoso_navigation.testing.waypoints:main',
            'test_controller = virtuoso_navigation.testing.controller:main',
            'test_forward = virtuoso_navigation.testing.forward:main',
            'test_backward = virtuoso_navigation.testing.backward:main'
        ],
    },
)
