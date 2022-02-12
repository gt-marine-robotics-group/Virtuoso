from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    processing = get_package_share_directory('virtuoso_processing')
    perception = get_package_share_directory('virtuoso_perception')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(processing, 'launch', 'lidar_processing.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(perception, 'launch', 'find_and_classify_buoys.launch.py'))),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'wamv/base_link', 'map']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_link_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'wamv/lidar_wamv_link', 'map']
        )
    ])