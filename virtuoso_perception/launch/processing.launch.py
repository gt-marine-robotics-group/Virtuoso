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

    ray_ground_classifier_param_file = (pkg_share, 
        '/config/', usv_config, '/ray_ground_classifier.yaml')

    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )

    lidar_processing_param_file = (pkg_share, '/config/', 
        usv_config, '/lidar_processing.yaml')
    camera_processing_param_file = (pkg_share, '/config/', usv_config,
        '/camera_processing.yaml')
    
    ld = list()

    ld.append(usv_arg)
    ld.append(ray_ground_classifier_param)

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='ground_filter',
            remappings=[('input', lidar_data['lidar_config']['all_lidar_base_topics'][0] + '/points') ]
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='euclidean_clustering'
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='self_filter_node.py',
            parameters=[lidar_processing_param_file],
        )
    )
    
    ld.append(
        Node(
            package='virtuoso_perception',
            executable='shore_filter_node.py',
            parameters=[lidar_processing_param_file]
        )
    )

    return LaunchDescription(ld)