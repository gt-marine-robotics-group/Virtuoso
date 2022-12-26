from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='virtuoso_autonomy',
            executable='roboboat_channel_nav'
        )
    ])