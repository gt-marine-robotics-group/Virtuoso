from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import sys
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]
    
    camera_data = None
    with open(f'{pkg_share}/config/{usv_config_str}/camera_config.yaml', 'r') as stream:
        camera_data = yaml.safe_load(stream)

    ld = []

    topic = camera_data['camera_config']['bow_camera_base_topics'][0]

    ld.append(usv_arg)

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='yolo_node.py',
            name=f'perception_YOLO_{topic[topic.rfind("/") + 1:]}',
            parameters=[
                {'usv': usv_config_str}
            ],
            remappings=[
                ('input', topic + '/image_raw'),
                ('results', topic + '/yolo_results'),
                ('yolo_debug', topic + '/yolo_debug')
            ]
        )
    )

    return LaunchDescription(ld)