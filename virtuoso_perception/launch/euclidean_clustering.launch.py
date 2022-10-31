from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    euclidean_clustering_params_file = os.path.join(pkg_share, 'param', 'euclidean_clustering.param.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'euclidean_clustering_params_file',
            default_value=euclidean_clustering_params_file
        ),
        Node(
            package='euclidean_cluster_nodes',
            executable='euclidean_cluster_node_exe',
            parameters=[LaunchConfiguration('euclidean_clustering_params_file')],
            remappings=[('/points_in', '/local_costmap/voxel_grid')]
            # remappings=[('/points_in', '/points_self_filtered')]
            # remappings=[('/points_in', '/points_shore_filtered')]
        )
    ])