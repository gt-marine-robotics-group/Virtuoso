from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    code_params_file = (pkg_share, '/config/', usv_config, '/code.yaml')

    return LaunchDescription([
        usv_arg,

        Node(
            package='virtuoso_perception',
            executable='scan_code_node.py',
            parameters=[code_params_file]
        ) 
    ])