from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='virtuoso_localization',
            executable='keyboard_listener'
        )
    ])