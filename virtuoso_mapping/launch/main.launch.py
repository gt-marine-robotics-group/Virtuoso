from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    processing_pkg = get_package_share_directory('virtuoso_processing')
    localization_pkg = get_package_share_directory('virtuoso_localization')
    navigation_pkg = get_package_share_directory('virtuoso_navigation')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(processing_pkg, 'launch/main.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(localization_pkg, 'launch/ekf.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(navigation_pkg, 'launch/astar.launch.py')))
    ])