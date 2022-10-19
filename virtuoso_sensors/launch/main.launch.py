from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_sensors')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/lidar.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/gps.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/imu.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/camera.launch.py')))
    ])