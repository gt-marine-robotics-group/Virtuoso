from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource('virtuoso_processing/launch/camera_processing.launch.py')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource('virtuoso_processing/launch/lidar_processing.launch.py'))
    ])