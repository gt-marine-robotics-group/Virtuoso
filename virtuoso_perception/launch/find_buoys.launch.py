from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import sys
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_perception')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    buoys_param_file = (pkg_share, '/config/', usv_config, '/buoys.yaml')
    stereo_param_file = (pkg_share, '/config/', usv_config, '/stereo.yaml')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]

    camera_data = None
    with open(f'{pkg_share}/config/{usv_config_str}/camera_config.yaml', 'r') as stream:
        camera_data = yaml.safe_load(stream)

    ld = list()

    ld.append(usv_arg)

    ld.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/euclidean_clustering.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='find_buoys',
            parameters=[buoys_param_file]
        )
    )

    for topic in camera_data['camera_config']['bow_camera_base_topics']:
        ld.append(
            Node(
                package='virtuoso_perception',
                executable='buoy_filter',
                name=f'perception_buoy_filter_{topic[topic.rfind("/") + 1:]}',
                parameters=[
                    {'base_topic': topic}
                ]
            )
        )
        ld.append(
            Node(
                package='virtuoso_perception',
                executable='resize',
                name=f'perception_resize_{topic[topic.rfind("/") + 1:]}',
                parameters=[
                    {'base_topic': topic}
                ]
            )
        )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='grayscale'
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='buoy_stereo',
            parameters=[
                {'base_topics': camera_data['camera_config']['bow_camera_base_topics']},
                {'frames': camera_data['camera_config']['bow_camera_frames']}, 
                stereo_param_file
            ]
        )
    )

    ld.append(
        Node(
            package='virtuoso_perception',
            executable='channel'
        )
    )
        
    return LaunchDescription(ld)