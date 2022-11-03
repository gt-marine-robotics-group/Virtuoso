from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description(): 

    pkg_share = get_package_share_directory('virtuoso_processing')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    # ray_ground_classifier_param_file = os.path.join(pkg_share, 'param/ray_ground_classifier.param.yaml')
    ray_ground_classifier_param_file = (pkg_share, 
        '/config/', usv_config, '/ray_ground_classifier.yaml')

    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )

    # voxel_grid_node_param_file = os.path.join(pkg_share, 'param', 'voxel_grid_node.param.yaml')
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
        ray_ground_classifier_param,

        # running ground filter on raw LIDAR data
        Node(
            package='ray_ground_classifier_nodes',
            executable='ray_ground_classifier_cloud_node_exe',
            parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
            remappings=[("points_in", "wamv/sensors/lidars/lidar_wamv/points")] # points_in comes from raw LIDAR data
        ),
        Node(
            package='voxel_grid_nodes',
            executable='voxel_grid_node_exe',
            parameters=[LaunchConfiguration('voxel_grid_node_param_file')],
            remappings=[
                ('points_in', '/local_costmap/voxel_grid'),
                ('points_downsampled', '/processing/super_voxels')
            ]
        ),

        Node(
            package='virtuoso_processing',
            executable='self_filter'
        ),
        Node(
            package='virtuoso_processing',
            executable='shore_filter'
        ),
    ])