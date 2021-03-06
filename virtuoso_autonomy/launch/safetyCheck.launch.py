from struct import pack
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    processing = get_package_share_directory('virtuoso_processing')
    navigation = get_package_share_directory('virtuoso_navigation')
    localization = get_package_share_directory('virtuoso_localization')
    perception = get_package_share_directory('virtuoso_perception')
    controller = get_package_share_directory('virtuoso_controller')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(processing, 'launch', 'main.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(localization, 'launch', 'ekf.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'main.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(perception, 'launch', 'find_buoys.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(controller, 'launch', 'main.launch.py'))),

        Node(
            package='virtuoso_autonomy',
            executable='robotX_safetyCheck'
        )
    ])
