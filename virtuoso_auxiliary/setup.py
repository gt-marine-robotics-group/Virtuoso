from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'virtuoso_auxiliary'

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
    maintainer='mroglan',
    maintainer_email='manueljoseph113@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            # Testing
            'test_ball_shooter = virtuoso_auxiliary.testing.ball_shooter_node:main'
        ],
    },
)
