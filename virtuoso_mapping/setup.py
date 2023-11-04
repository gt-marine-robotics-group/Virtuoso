from setuptools import setup
import os
from glob import glob

package_name = 'virtuoso_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'buoy_map = virtuoso_mapping.buoy_map_node:main'
        ],
    },
)
