from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import sys
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_autonomy')
    perception_share = get_package_share_directory('virtuoso_perception')

    usv_arg = DeclareLaunchArgument('usv')

    usv_config = LaunchConfiguration('usv')

    channel_nav_config_file = (pkg_share, '/config/', usv_config, '/channel_nav.yaml')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]
    
    camera_data = None
    with open(f'{perception_share}/config/{usv_config_str}/camera_config.yaml', 'r') as stream:
        camera_data = yaml.safe_load(stream)

    topic = camera_data['camera_config']['bow_camera_base_topics'][0]

    return LaunchDescription([
        usv_arg, 

        Node(
            package='virtuoso_autonomy',
            executable='roboboat_channel_nav',
            parameters=[channel_nav_config_file],
            remappings=[
                ('yolo_results', f'{topic}/yolo_results'),
                ('camera_info', f'{topic}/camera_info')
            ]
        )
    ])