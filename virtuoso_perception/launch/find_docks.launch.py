from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/euclidean_clustering.launch.py'))),
        Node(
            package='virtuoso_perception',
            executable='find_docks'
        ) 
    ])