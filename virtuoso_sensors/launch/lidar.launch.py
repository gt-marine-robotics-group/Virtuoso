from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    sensors = get_package_share_directory('virtuoso_sensors')

    config_file = os.path.join(sensors, 'config', 'vlp16.yml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vlp16_node_param_file',
            default_value=config_file
        ),
        Node(
            package='velodyne_nodes',
            namespace='lidar_front',
            executable='velodyne_cloud_node_exe',
            parameters=[LaunchConfiguration('vlp16_node_param_file')],
            remappings=[("topic", "points_xyzi")],
            arguments=["--model", "vlp16"]
        ),
        Node(
            package='virtuoso_sensors',
            executable='lidar_republish'
        )
    ])