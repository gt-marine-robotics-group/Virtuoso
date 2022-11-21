from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'virtuoso_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        (os.path.join('share', package_name, 'config/vrx_robotx'), 
            glob(os.path.join('config', 'vrx_robotx', '*.yaml'))),

        (os.path.join('share', package_name, 'config/robotx'),
            glob(os.path.join('config', 'robotx', '*.yaml'))),

        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
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
            'find_buoys = virtuoso_perception.buoys.find_buoys_node:main',
            'classify_buoys = virtuoso_perception.buoys.classify_buoys:main',
            'scan_code = virtuoso_perception.code.scan_code_node:main',
            'find_dock_codes = virtuoso_perception.dock.find_dock_codes_node:main',
            'find_dock_entrances = virtuoso_perception.dock.find_dock_entrances_node:main'
        ],
    },
)
