from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os

def generate_launch_description():

    sensors = get_package_share_directory('virtuoso_sensors')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    return LaunchDescription([
        usv_arg,
        Node(
            package='virtuoso_sensors',
            executable='f9p_gps_republish'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_baselink',
            arguments=['1.17', '0.06', '1.245', '0.32', '0', '0.0', 'wamv/base_link', 'imu_frame'],
            condition=IfCondition(PythonExpression(["'", usv_config, "' == 'robotx'"]))
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_to_baselink',
            arguments=['1.17', '-0.1', '1.245', '0', '0', '0', 'wamv/base_link', 'ubx'],
            condition=IfCondition(PythonExpression(["'", usv_config, "' == 'robotx'"]))
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_baselink',
            arguments=['0.0', '0.0', '0', '-0.095', '0', '0.0', 'base_link', 'imu_frame'],
            condition=IfCondition(PythonExpression(["'", usv_config, "' == 'roboboat'"]))
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_to_baselink',
            arguments=['1.0', '0.0', '0', '0', '0', '0', 'base_link', 'ubx'],
            condition=IfCondition(PythonExpression(["'", usv_config, "' == 'roboboat'"]))
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sensors, 'launch', 'f9p.launch.py'))),
    ])
