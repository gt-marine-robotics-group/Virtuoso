from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# Using ray_ground_classifier from Autoware.auto for ground filter

def generate_launch_description(): 

    pkg_share = get_package_share_directory('virtuoso_processing')

    ray_ground_classifier_param_file = os.path.join(pkg_share, 'param/ray_ground_classifier.param.yaml')

    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )

    return LaunchDescription([

        # running ground filter on raw LIDAR data
        ray_ground_classifier_param,
        Node(
            package='ray_ground_classifier_nodes',
            executable='ray_ground_classifier_cloud_node_exe',
            parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
            remappings=[("points_in", "wamv/sensors/lidars/lidar_wamv/points")] # points_in comes from raw LIDAR data
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