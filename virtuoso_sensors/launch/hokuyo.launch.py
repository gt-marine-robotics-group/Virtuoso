from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_sensors')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    urg_params_file = (pkg_share, '/config/urg.yml')

    return LaunchDescription([
        usv_arg,

        Node(
            package='urg_node',
            executable='urg_node_driver',
            parameters=[urg_params_file]
        )
    ])
