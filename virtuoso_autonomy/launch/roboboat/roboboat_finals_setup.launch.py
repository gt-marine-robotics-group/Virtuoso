from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument('sim_time', default_value='False')
    usv_arg = DeclareLaunchArgument('usv')

    sim_time_config = LaunchConfiguration('sim_time', default='False')
    usv_config = LaunchConfiguration('usv')

    navigation = get_package_share_directory('virtuoso_navigation')
    localization = get_package_share_directory('virtuoso_localization')
    perception = get_package_share_directory('virtuoso_perception')
    controller = get_package_share_directory('virtuoso_controller')
    auxiliary = get_package_share_directory('virtuoso_auxiliary')

    return LaunchDescription([
        sim_time_arg,
        usv_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization, 'launch', 'ekf.launch.py')),
            launch_arguments= {'sim_time': sim_time_config, 'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'main.launch.py')),
            launch_arguments={'sim_time': sim_time_config, 'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'multi_tasks_waypoint_player.launch.py')),
            launch_arguments={'sim_time': sim_time_config, 'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(perception, 'launch', 'find_buoys.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(controller, 'launch', 'main.launch.py')),
            launch_arguments={'sim_time': sim_time_config, 'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(auxiliary, 'launch', 'main.launch.py')),
            launch_arguments={'usv': usv_config, 'sim_time': sim_time_config}.items()
        )
    ])