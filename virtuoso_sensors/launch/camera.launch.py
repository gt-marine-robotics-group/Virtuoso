from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe'
        ),
        Node(
            package='virtuoso_sensors',
            executable='camera_republish'
        )
    ])