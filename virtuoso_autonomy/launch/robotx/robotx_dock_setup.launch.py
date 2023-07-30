from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    usv = DeclareLaunchArgument('usv')

    usv_config = LaunchConfiguration('usv')

    navigation = get_package_share_directory('virtuoso_navigation')
    localization = get_package_share_directory('virtuoso_localization')
    perception = get_package_share_directory('virtuoso_perception')
    controller = get_package_share_directory('virtuoso_controller')

    return LaunchDescription([
        usv,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization, 'launch', 'ekf.launch.py')),
            launch_arguments= {'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'main.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(perception, 'launch', 'find_docks.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(controller, 'launch', 'main.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
    ])