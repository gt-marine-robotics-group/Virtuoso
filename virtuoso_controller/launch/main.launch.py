import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

pkg_share = FindPackageShare(package='virtuoso_controller').find('virtuoso_controller')
#robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 

virtuoso_params_file = os.path.join(pkg_share, 'config/pidgains.yml') 

#virtuoso_params_file = os.path.join(
#  get_package_share_directory('virtuoso_controller'),
#  'config',
#  'pid_gains.yml'
#)


def generate_launch_description():

    sim_time = DeclareLaunchArgument('sim_time', default_value='False')

    sim_time_config = LaunchConfiguration('sim_time', default='False')

    return LaunchDescription([
        sim_time,
        Node(
            package='virtuoso_controller',
            executable='basic_pid',
            parameters = [virtuoso_params_file]
        ),
        Node(
            package='virtuoso_controller',
            executable='velocity_pid',
            parameters = [virtuoso_params_file]
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
