from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os
import sys
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    camera_processing_param_file = (pkg_share, '/config/', usv_config,
        '/camera_processing.yaml')
    dock_codes_param_file = (pkg_share, '/config/', usv_config, '/dock_codes.yaml')
    dock_posts_param_file = (pkg_share, '/config/', usv_config, '/dock_posts.yaml')
    stereo_param_file = (pkg_share, '/config/', usv_config, '/stereo.yaml')

    voxel_grid_node_param_file = (pkg_share,
        '/config/', usv_config, '/voxel_grid.yaml')

    voxel_grid_node_param = DeclareLaunchArgument(
        'voxel_grid_node_param_file',
        default_value=voxel_grid_node_param_file,
        description='Path to config file for Voxel Grid Node'
    )

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]

    camera_data = None
    with open(f'{pkg_share}/config/{usv_config_str}/camera_config.yaml', 'r') as stream:
        camera_data = yaml.safe_load(stream)

    ld = list()

    ld.append(usv_arg)
    ld.append(voxel_grid_node_param)

    ld.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch/processing.launch.py')
            ),
            launch_arguments={'usv': usv_config}.items()
        )
    )

    for base_topic in camera_data['camera_config']['bow_camera_base_topics']:
        ld.append(
            Node(
                package='virtuoso_perception',
                executable='find_dock_codes',
                name=f'find_dock_codes_{base_topic[base_topic.rfind("/") + 1:]}',
                parameters=[
                    {'camera_base_topic': base_topic},
                    dock_codes_param_file,
                    camera_processing_param_file
                ]
            )
        )
        ld.append(
            Node(
                package='virtuoso_perception',
                executable='find_dock_posts',
                name=f'find_dock_posts_{base_topic[base_topic.rfind("/") + 1:]}',
                parameters=[
                    {'camera_base_topic': base_topic},
                    dock_posts_param_file,
                    camera_processing_param_file
                ]
            )
        )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='dock_stereo',
            parameters=[
                {'base_topics': camera_data['camera_config']['bow_camera_base_topics']},
                {'frames': camera_data['camera_config']['bow_camera_frames']}, 
                stereo_param_file
            ]
        )
    )

    # ld.append(
    #     Node(
    #         package='virtuoso_perception',
    #         executable='find_dock_entrances',
    #         parameters=[dock_param_file]
    #     )
    # )

    return LaunchDescription(ld)
    
