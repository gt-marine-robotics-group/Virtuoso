from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# Using ray_ground_classifier from Autoware.auto
# Using STVL to downsample and generate Costmap

def generate_launch_description(): 

    pkg_share = get_package_share_directory('virtuoso_processing')

    bringup_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    nav2_params_file = os.path.join(pkg_share, 'param', 'stvl.param.yaml')

    ray_ground_classifier_param_file = os.path.join(pkg_share, 'param/ray_ground_classifier.param.yaml')

    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )

    return LaunchDescription([
        ray_ground_classifier_param,
        Node(
            package='ray_ground_classifier_nodes',
            executable='ray_ground_classifier_cloud_node_exe',
            parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
            remappings=[("points_in", "/horizontal_laser_3d")] # points_in comes from raw LIDAR data
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_launch_file),launch_arguments={'params_file': nav2_params_file}.items()),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', arguments=[nav2_params_file]),
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
        ),
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='horizontal_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'horizontal_vlp16_link', 'base_link'],
        )
    ])