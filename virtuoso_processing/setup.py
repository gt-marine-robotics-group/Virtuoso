from setuptools import setup

package_name = 'virtuoso_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name, ['launch/lidar_processing.launch.py']),
        ('share/' + package_name, ['launch/camera_processing.launch.py']),
        ('share/' + package_name, ['launch/main.launch.py'])
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
            'grayscale = virtuoso_processing.grayscale:main',
            'downscale = virtuoso_processing.downscale:main'
        ],
    },
)
