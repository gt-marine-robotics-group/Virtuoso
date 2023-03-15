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
    dock_param_file = (pkg_share, '/config/', usv_config, '/dock.yaml')

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
    
    return LaunchDescription([
        usv_arg,
        voxel_grid_node_param,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch/processing.launch.py')
            ),
            launch_arguments={'usv': usv_config}.items()
        ),
        Node(
            package='voxel_grid_nodes',
            executable='voxel_grid_node_exe',
            parameters=[LaunchConfiguration('voxel_grid_node_param_file')],
            remappings=[
                ('points_in', '/perception/lidar/points_shore_filtered'),
                ('points_downsampled', '/perception/voxels')
            ]
        ),

        Node(
            package='virtuoso_perception',
            executable='find_dock_codes',
            parameters=[
                {'camera_base_topic': camera_data['camera_config']['all_camera_base_topics'][camera_data['camera_config']['dock_camera_index']]},
                dock_param_file,
                camera_processing_param_file
            ]
        ),
        Node(
            package='virtuoso_perception',
            executable='find_dock_entrances',
            parameters=[dock_param_file]
        )
    ])