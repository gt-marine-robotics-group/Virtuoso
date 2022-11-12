from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    return LaunchDescription([
        usv_arg,

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[
                {'video_device': '/dev/video2'}
            ]
        ),
        Node(
            package='virtuoso_sensors',
            executable='camera_republish'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='leftcam_to_baselink',
            arguments=['1.17', '-0.18', '1.245', '0', '0', '0', 'wamv/base_link', 'wamv/front_left_camera_link_optical'],
            condition=IfCondition(PythonExpression(["'", usv_config, "' == 'robotx'"]))
        )
    ])