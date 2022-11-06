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
        (os.path.join('share', package_name, 'launch', 'vrx'), glob(os.path.join('launch', 'vrx', '*.launch.py')))
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
            'vrx_mission_interpreter = virtuoso_autonomy.vrx.mission_interpreter:main',
            'vrx_perception = virtuoso_autonomy.vrx.perception.main:main',
            'vrx_station_keeping = virtuoso_autonomy.vrx.station_keeping.main:main',
            'vrx_wayfinding = virtuoso_autonomy.vrx.wayfinding.main:main',

            'robotX_safetyCheck = virtuoso_autonomy.robotx.safety_check.main:main',
            'robotX_gymkhana = virtuoso_autonomy.robotx.gymkhana.main:main',
            'robotX_enter_exit = virtuoso_autonomy.robotx.enter_and_exit.main:main',
            'robotX_heartbeat = virtuoso_autonomy.robotx.heartbeat.main:main',
            'robotX_scan_code = virtuoso_autonomy.robotx.scan_code.main:main',
            'robotX_docking = virtuoso_autonomy.robotx.docking.main:main'
        ],
    },
)
