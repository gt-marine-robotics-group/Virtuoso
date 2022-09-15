import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import rclpy

pkg_share = FindPackageShare(package='virtuoso_localization').find('virtuoso_localization')
robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='virtuoso_controller',
            executable='basic_pid',
        ),
        Node(
            package='virtuoso_controller',
            executable='velocity_pid',
        ),
        Node(
            package='virtuoso_controller',
            executable='motor_cmd_generator',
        ),
    ])
