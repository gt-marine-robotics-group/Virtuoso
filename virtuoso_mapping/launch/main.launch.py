from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    pkg_share = get_package_share_directory('virtuoso_mapping')

    occupancy_map_param_file = (pkg_share, '/config/', usv_config, '/occupancy_map_generator.yaml')

    return LaunchDescription([
        usv_arg,

        Node(
            package='virtuoso_mapping',
            executable='occupancy_map_generator_node.py',
            parameters=[occupancy_map_param_file]
        ),
        Node(
            package='virtuoso_mapping',
            executable='pcl_to_map_frame'
        )
    ])