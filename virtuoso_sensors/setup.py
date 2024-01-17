import os
from glob import glob
from setuptools import setup

package_name = 'virtuoso_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yml')))
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
            'f9p_gps_republish = virtuoso_sensors.f9p_gps_republish:main',
            'gx3_republish = virtuoso_sensors.gx3_republish:main',
            'lidar_republish = virtuoso_sensors.lidar_republish:main',
            'camera_info = virtuoso_sensors.camera_info_node:main',
            'camera_republish = virtuoso_sensors.camera_republish:main',

            'laser_to_pcd = virtuoso_sensors.laser_to_pcd_node:main'
        ],
    },
)
