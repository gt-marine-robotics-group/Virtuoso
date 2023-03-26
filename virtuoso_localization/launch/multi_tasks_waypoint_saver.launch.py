from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_localization')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    multi_params = (pkg_share, '/config/', usv_config, '/multi_tasks_waypoint_saver.yaml')

    return LaunchDescription([
        usv_arg,

        Node(
            package='virtuoso_localization',
            executable='keyboard_listener'
        ),
        Node(
            package='virtuoso_localization',
            executable='multi_tasks_waypoint_saver',
            parameters=[multi_params]
        )
    ])