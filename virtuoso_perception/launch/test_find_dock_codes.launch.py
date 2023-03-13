from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import sys
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    camera_processing_param_file = (pkg_share, '/config/', usv_config,
        '/camera_processing.yaml')
    dock_param_file = (pkg_share, '/config/', usv_config, '/dock.yaml')
    
    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]

    camera_data = None
    with open(f'{pkg_share}/config/{usv_config_str}/camera_config.yaml', 'r') as stream:
        camera_data = yaml.safe_load(stream)
    
    return LaunchDescription([
        Node(
            package='virtuoso_perception',
            executable='find_dock_codes',
            parameters=[
                {'camera_base_topic': camera_data['camera_config']['all_camera_base_topics'][camera_data['camera_config']['dock_camera_index']]},
                dock_param_file,
                camera_processing_param_file
            ]
        )
    ])