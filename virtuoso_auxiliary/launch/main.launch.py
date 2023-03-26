import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_auxiliary')

    sim_time_arg = DeclareLaunchArgument('sim_time')
    sim_time_config = LaunchConfiguration('sim_time', default='False')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    aux_serial_params = (pkg_share, '/config/', usv_config, '/aux_serial.yaml')

    return LaunchDescription([
        usv_arg,
        sim_time_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'ball_shooter.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'water_shooter.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        Node(
            package='virtuoso_auxiliary',
            executable='aux_serial',
            parameters=[aux_serial_params],
            condition=UnlessCondition(sim_time_config)
        )
    ])