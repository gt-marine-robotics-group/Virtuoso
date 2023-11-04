from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_mapping')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    costmap_2d_file_path = (pkg_share, '/config/', usv_config, '/costmap_2d.yaml')

    return LaunchDescription([
        usv_arg,

        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            parameters=[costmap_2d_file_path]
        )
    ])