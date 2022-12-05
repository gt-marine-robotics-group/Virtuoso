from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='virtuoso_processing',
            executable='buoy_color_filter'
        ),
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
        ),
        Node(
            package='virtuoso_processing',
            executable='stereo_filter'
        )
    ])