from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    usv = DeclareLaunchArgument('usv')

    usv_config = LaunchConfiguration('usv')

    perception = get_package_share_directory('virtuoso_perception')
    navigation = get_package_share_directory('virtuoso_navigation')
    localization = get_package_share_directory('virtuoso_localization')
    controller = get_package_share_directory('virtuoso_controller')
    auxiliary = get_package_share_directory('virtuoso_auxiliary')

    return LaunchDescription([
        usv,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception, 'launch', 'processing.launch.py')
            ),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization, 'launch', 'ekf.launch.py')),
            launch_arguments= {'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'main.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'multi_tasks_waypoint_player.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(controller, 'launch', 'main.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(auxiliary, 'launch', 'main.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        )
    ])
