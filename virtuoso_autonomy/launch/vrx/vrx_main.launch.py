from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    navigation = get_package_share_directory('virtuoso_navigation')
    localization = get_package_share_directory('virtuoso_localization')
    perception = get_package_share_directory('virtuoso_perception')
    controller = get_package_share_directory('virtuoso_controller')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(localization, 'launch', 'ekf.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'main.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(perception, 'launch', 'find_and_classify_buoys.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(controller, 'launch', 'main.launch.py'))),
        Node(
            package='virtuoso_autonomy',
            executable='vrx_mission_interpreter'
        ),
        Node(
            package='virtuoso_autonomy',
            executable='vrx_station_keeping'
        ),
        Node(
            package='virtuoso_autonomy',
            executable='vrx_perception'
        ),
        Node(
            package='virtuoso_autonomy',
            executable='vrx_wayfinding'
        )
    ])
