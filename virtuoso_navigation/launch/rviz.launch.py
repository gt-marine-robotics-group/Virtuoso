import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_navigation')

    config_file = os.path.join(pkg_share, 'rviz', 'view.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', config_file],
            output='screen'
        )
    ])