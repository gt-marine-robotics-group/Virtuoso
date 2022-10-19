import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import rclpy

pkg_share = FindPackageShare(package='virtuoso_localization').find('virtuoso_localization')
robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 


def generate_launch_description():

    sim_time = DeclareLaunchArgument('sim_time', default_value='False')

    sim_time_config = LaunchConfiguration('sim_time', default='False')

    return LaunchDescription([
        sim_time,
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
            parameters=[
                {'sim_time', sim_time_config}
            ]
        ),
        Node(
            package='virtuoso_controller',
            executable='cmd_vel_generator',
       )
    ])
