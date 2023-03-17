from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'virtuoso_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch', 'robotx'), glob(os.path.join('launch', 'robotx', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch', 'vrx'), glob(os.path.join('launch', 'vrx', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch', 'roboboat'), glob(os.path.join('launch', 'roboboat', '*.launch.py'))),

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
    maintainer_email='gtmrg@TODO.TODO',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # VRX LAUNCHES
            'vrx_mission_interpreter = virtuoso_autonomy.vrx.mission_interpreter:main',
            'vrx_perception = virtuoso_autonomy.vrx.perception.main:main',
            'vrx_station_keeping = virtuoso_autonomy.vrx.station_keeping.main:main',
            'vrx_wayfinding = virtuoso_autonomy.vrx.wayfinding.main:main',


            # ROBOTX LAUNCHES
            'robotX_safety_check = virtuoso_autonomy.robotx.safety_check.safety_check_node:main',
            'robotX_gymkhana = virtuoso_autonomy.robotx.gymkhana.gymkhana_node:main',
            'robotX_enter_exit = virtuoso_autonomy.robotx.enter_and_exit.enter_exit_node:main',
            'robotX_scan_code = virtuoso_autonomy.robotx.scan_code.scan_code_node:main',
            'robotX_docking = virtuoso_autonomy.robotx.docking.docking_node:main',
            'robotX_wildlife_encounter = virtuoso_autonomy.robotx.wildlife_encounter.wildlife_encounter_node:main',


            #ROBOBOAT LAUNCHES
            'roboboat_channel_nav = virtuoso_autonomy.roboboat.channel_nav.channel_nav_node:main',
            'roboboat_safety_check = virtuoso_autonomy.roboboat.safety_check.safety_check_node:main',
            'roboboat_loop = virtuoso_autonomy.roboboat.loop.loop_node:main',
            'roboboat_docking = virtuoso_autonomy.roboboat.docking.docking_node:main'
        ],
    },
)
