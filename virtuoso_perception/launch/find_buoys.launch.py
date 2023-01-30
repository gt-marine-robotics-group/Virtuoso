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

    buoys_param_file = (pkg_share, '/config/', usv_config, '/buoys.yaml')
    stereo_param_file = (pkg_share, '/config/', usv_config, '/stereo.yaml')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]

    camera_data = None
    with open(f'{pkg_share}/config/{usv_config_str}/camera_config.yaml', 'r') as stream:
        camera_data = yaml.safe_load(stream)
    
    buoys_data = None
    with open(f'{pkg_share}/config/{usv_config_str}/buoys.yaml', 'r') as stream:
        buoys_data = yaml.safe_load(stream)
    
    buoy_filter_params = []
    for key, value in buoys_data['perception_buoy_filter']['ros__parameters'].items():
        buoy_filter_params.append({key: value})

    ld = list()

    ld.append(usv_arg)

    ld.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch/processing.launch.py')
            ),
            launch_arguments={'usv': usv_config}.items()
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='buoy_lidar',
            parameters=[buoys_param_file]
        )
    )

    ld.append(Node(
        package='virtuoso_perception',
        executable='buoy_filter',
        parameters=[*buoy_filter_params]
    ))

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='buoy_stereo',
            parameters=[
                {'base_topics': camera_data['camera_config']['bow_camera_base_topics']},
                {'frames': camera_data['camera_config']['bow_camera_frames']}, 
                stereo_param_file
            ]
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='channel',
            parameters=[
                {'camera_frame': camera_data['camera_config']['bow_camera_frames'][0]}
            ]
        )
    )
        
    return LaunchDescription(ld)