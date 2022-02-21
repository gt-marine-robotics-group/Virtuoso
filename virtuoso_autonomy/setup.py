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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
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
            'mission_interpreter = virtuoso_autonomy.mission_interpreter:main',
            'perception = virtuoso_autonomy.tasks.perception.main:main',
            'station_keeping = virtuoso_autonomy.tasks.station_keeping.main:main'
        ],
    },
)
