from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    dock_param_file = (pkg_share, '/config/', usv_config, '/dock.yaml')

    voxel_grid_node_param_file = (pkg_share,
        '/config/', usv_config, '/voxel_grid_node.yaml')

    voxel_grid_node_param = DeclareLaunchArgument(
        'voxel_grid_node_param_file',
        default_value=voxel_grid_node_param_file,
        description='Path to config file for Voxel Grid Node'
    )
    
    return LaunchDescription([
        usv_arg,
        voxel_grid_node_param,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch/procesing.launch.py')
            ),
            launch_arguments={'usv': usv_config}.items()
        ),
        Node(
            package='voxel_grid_nodes',
            executable='voxel_grid_node_exe',
            parameters=[LaunchConfiguration('voxel_grid_node_param_file')],
            remappings=[
                ('points_in', '/local_costmap/voxel_grid'),
                ('points_downsampled', '/perception/voxel_voxels')
            ]
        ),

        Node(
            package='virtuoso_perception',
            executable='find_dock_codes',
            parameters=[dock_param_file]
        ),
        Node(
            package='virtuoso_perception',
            executable='find_dock_entrances',
            parameters=[dock_param_file]
        )
    ])