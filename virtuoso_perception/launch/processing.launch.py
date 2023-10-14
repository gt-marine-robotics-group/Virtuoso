from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import sys
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]
    
    camera_data = None
    with open(f'{pkg_share}/config/{usv_config_str}/camera_config.yaml', 'r') as stream:
        camera_data = yaml.safe_load(stream)
    
    lidar_data = None
    with open(f'{pkg_share}/config/{usv_config_str}/lidar_config.yaml', 'r') as stream:
        lidar_data = yaml.safe_load(stream)

    euclidean_clustering_params_file = (pkg_share, '/config/', usv_config, 
        '/euclidean_clustering.yaml')

    ground_filter_param_file = (pkg_share, '/config/', usv_config, '/ground_filter.yaml')

    lidar_processing_param_file = (pkg_share, '/config/', 
        usv_config, '/lidar_processing.yaml')
    camera_processing_param_file = (pkg_share, '/config/', usv_config,
        '/camera_processing.yaml')
    
    ld = list()

    ld.append(usv_arg)

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='ground_filter',
            parameters=[ground_filter_param_file],
            remappings=[('input', lidar_data['lidar_config']['all_lidar_base_topics'][0] + '/points') ]
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='euclidean_clustering',
            parameters=[euclidean_clustering_params_file]
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='self_filter',
            parameters=[lidar_processing_param_file],
        )
    )
    
    ld.append(
        Node(
            package='virtuoso_perception',
            executable='shore_filter',
            parameters=[lidar_processing_param_file]
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='voxels',
            parameters=[lidar_processing_param_file]
        )
    )

    return LaunchDescription(ld)