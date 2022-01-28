from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# Using ray_ground_classifier from Autoware.auto for ground filter
# Using STVL to downsample and generate Costmap

def generate_launch_description(): 

    pkg_share = get_package_share_directory('virtuoso_processing')

    # the launch file which we pass stvl in as a plugin an the params file
    bringup_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    nav2_params_file = os.path.join(pkg_share, 'param', 'nav2.param.yaml')

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

        # downsampling LIDAR data with STVL (also generates a costmap)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_launch_file),launch_arguments={'params_file': nav2_params_file}.items()),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', arguments=[nav2_params_file]),

        # Currently, state estimation only using odom frame for localization, so no difference between 
        # odom and map frame. Transformation being used for the costmaps.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map']
        )
    ])