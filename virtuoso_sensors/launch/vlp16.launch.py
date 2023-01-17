from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os

def generate_launch_description():

    velodyne_share = get_package_share_directory('velodyne')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    return LaunchDescription([
        usv_arg,

        Node(
            package='virtuoso_sensors',
            executable='lidar_republish'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(velodyne_share, 'launch', 'velodyne-all-nodes-VLP16-launch.py')
            )
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_to_baselink',
            arguments=['1.17', '-0.31', '1.245', '0', '0', '0', 'wamv/base_link', 'wamv/lidar_wamv_link'],
            condition=IfCondition(PythonExpression(["'", usv_config, "' == 'robotx'"]))
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidarfront_to_wamvlidar',
            # arguments=['0', '0', '0', '0', '-3.14159', '0', 'wamv/lidar_wamv_link', 'lidar_front']
            arguments=['0', '0', '0', '0', '0', '0', 'wamv/lidar_wamv_link', 'velodyne'],
            # arguments=['0', '0', '0', '0', '0', '3.14159', 'wamv/base_link', 'wamv/lidar_wamv_link']
            # arguments=['0', '0', '0', '3.14149', '0', '0', 'wamv/base_link', 'wamv/lidar_wamv_link']
            condition=IfCondition(PythonExpression(["'", usv_config, "' == 'robotx'"]))
        )
    ])