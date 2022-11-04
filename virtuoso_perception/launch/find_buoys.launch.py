from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    # must have virtuoso_processing lidar_processing launched as well for euclidean clustering to do anything
    return LaunchDescription([
        usv_arg, 

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/euclidean_clustering.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        Node(
            package='virtuoso_perception',
            executable='find_buoys'
        ) 
    ])