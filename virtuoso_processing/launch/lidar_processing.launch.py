from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(): 

    pkg_share = get_package_share_directory('virtuoso_processing')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    ray_ground_classifier_param_file = (pkg_share, 
        '/config/', usv_config, '/ray_ground_classifier.yaml')

    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )

    processing_param_file = (pkg_share, '/config/', usv_config, '/lidar_processing.yaml')

    return LaunchDescription([
        usv_arg,
        ray_ground_classifier_param,

        Node(
            package='ray_ground_classifier_nodes',
            executable='ray_ground_classifier_cloud_node_exe',
            parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
            remappings=[("points_in", "wamv/sensors/lidars/lidar_wamv/points")]
        ),

        Node(
            package='virtuoso_processing',
            executable='self_filter',
            parameters=[processing_param_file]
        ),
        Node(
            package='virtuoso_processing',
            executable='shore_filter',
            parameters=[processing_param_file]
        ),
    ])