from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='virtuoso_mapping',
            executable='occupancy_map_generator_node.py'
        ),
        Node(
            package='virtuoso_mapping',
            executable='pcl_to_map_frame'
        )
    ])