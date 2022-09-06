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
    autonomy = get_package_share_directory('virtuoso_autonomy')

    return LaunchDescription([
        #IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(processing, 'launch', 'main.launch.py'))),
        Node(
            package='virtuoso_autonomy',
            executable='f9p_gps_republish'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_baselink',
            arguments=['0', '0', '0', '0', '0', '0', 'wamv/base_link', 'imu_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_to_baselink',
            arguments=['0', '0', '0', '0', '0', '0', 'wamv/base_link', 'ubx']
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(autonomy, 'launch', 'f9p.launch.py'))),
    ])
