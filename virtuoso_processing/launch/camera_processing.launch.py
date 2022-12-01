from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='virtuoso_processing',
            executable='downscale'
        ),
        Node(
            package='virtuoso_processing',
            executable='grayscale'
        ),
        Node(
            package='virtuoso_processing',
            executable='stereo'
        )
    ])