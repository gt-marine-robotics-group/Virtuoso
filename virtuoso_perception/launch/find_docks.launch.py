from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    voxel_grid_node_param_file = os.path.join(pkg_share, 'param', 'voxel_grid_node.param.yaml')

    voxel_grid_node_param = DeclareLaunchArgument(
        'voxel_grid_node_param_file',
        default_value=voxel_grid_node_param_file,
        description='Path to config file for Voxel Grid Node'
    )
    return LaunchDescription([
        voxel_grid_node_param,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/euclidean_clustering.launch.py'))),
        Node(
            package='virtuoso_perception',
            executable='find_docks'
        ),
        Node(
            package='voxel_grid_nodes',
            executable='voxel_grid_node_exe',
            parameters=[LaunchConfiguration('voxel_grid_node_param_file')],
            remappings=[('points_in', '/local_costmap/voxel_grid')]
        )
    ])