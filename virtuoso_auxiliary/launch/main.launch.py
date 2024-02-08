import sys
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_auxiliary')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]

    is_sim_time = bool('vrx' in usv_config_str)

    aux_serial_params = (pkg_share, '/config/', usv_config, '/aux_serial.yaml')

    ld = [
        usv_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'ball_shooter.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'water_shooter.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        )
    ]

    # if not is_sim_time:
    #     ld.append(
    #         Node(
    #             package='virtuoso_auxiliary',
    #             executable='aux_serial',
    #             parameters=[aux_serial_params]
    #         )
    #     )

    return LaunchDescription(ld)