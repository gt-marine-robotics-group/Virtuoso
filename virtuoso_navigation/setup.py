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
            'get_points = virtuoso_navigation.get_points:main',
            'astar = virtuoso_navigation.astar:main',
            'astar_tests = virtuoso_navigation.tests.astar:main',
            'get_nodes_tests = virtuoso_navigation.tests.get_nodes:main',
            'create_path_tests = virtuoso_navigation.tests.create_path:main',
            'set_goal = virtuoso_navigation.testing.set_goal:main'
        ],
    },
)
