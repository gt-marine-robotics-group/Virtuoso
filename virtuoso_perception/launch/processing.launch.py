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
        DeclareLaunchArgument(
            'euclidean_clustering_params_file',
            default_value=euclidean_clustering_params_file
        ),
    )

    ld.append(
        Node(
            package='euclidean_cluster_nodes',
            executable='euclidean_cluster_node_exe',
            parameters=[LaunchConfiguration('euclidean_clustering_params_file')],
            remappings=[('/points_in', '/local_costmap/voxel_grid')]
        )
    )

    ld.append(
        Node(
            package='ray_ground_classifier_nodes',
            executable='ray_ground_classifier_cloud_node_exe',
            parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
            # remappings=[("points_in", "wamv/sensors/lidars/lidar_wamv/points")]
            remappings=[('points_in', lidar_data['lidar_config']['all_lidar_base_topics'][0] + '/points')]
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='self_filter',
            parameters=[lidar_processing_param_file]
        )
    )
    
    ld.append(
        Node(
            package='virtuoso_perception',
            executable='shore_filter',
            parameters=[lidar_processing_param_file]
        )
    )

    for topic in camera_data['camera_config']['bow_camera_base_topics']:
        ld.append(
            Node(
                package='virtuoso_perception',
                executable='resize',
                name=f'perception_resize_{topic[topic.rfind("/") + 1:]}',
                parameters=[
                    {'base_topic': topic},
                    camera_processing_param_file
                ]
            )
        )
    ld.append(Node(
        package='virtuoso_perception',
        executable='noise_filter',
        parameters=[camera_processing_param_file]
    ))

    return LaunchDescription(ld)