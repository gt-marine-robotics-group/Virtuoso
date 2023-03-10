from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import sys
import yaml

# BASE_CAM_TOPIC = '/wamv/sensors/cameras/front_left_camera'
BASE_CAM_TOPIC = '/cameras/front_left_camera'
RED = True
GREEN = True
BLACK = True
YELLOW = True

def generate_launch_description():

    null_red = {
        'lower1': [0,0,0],
        'upper1': [0,0,0],
        'lower2': [0,0,0],
        'upper2': [0,0,0],
    }

    null_color = {
        'lower': [0,0,0],
        'upper': [0,0,0]
    }

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    pkg_share = get_package_share_directory('virtuoso_perception')

    camera_processing_param_file = (pkg_share, '/config/', usv_config,
        '/camera_processing.yaml')
    
    buoy_param_file = (pkg_share, '/config/', usv_config, '/buoys.yaml')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]

    buoy_data = None
    with open(f'{pkg_share}/config/{usv_config_str}/buoys.yaml', 'r') as stream:
        buoy_data = yaml.safe_load(stream)

    filter_bounds = buoy_data['perception_buoy_filter']['ros__parameters']['filter_bounds']

    new_filter_bounds = dict()
    if (RED):
        new_filter_bounds['red'] = filter_bounds['red']
    else:
        new_filter_bounds['red'] = null_red
    if (GREEN):
        new_filter_bounds['green'] = filter_bounds['green']
    else:
        new_filter_bounds['green'] = null_color
    if (BLACK):
        new_filter_bounds['black'] = filter_bounds['black']
    else:
        new_filter_bounds['black'] = null_color
    if (YELLOW):
        new_filter_bounds['yellow'] = filter_bounds['yellow']
    else:
        new_filter_bounds['yellow'] = null_color
    
    return LaunchDescription([
        usv_arg,

        Node(
            package='virtuoso_perception',
            executable='resize',
            name=f'perception_resize_{BASE_CAM_TOPIC[BASE_CAM_TOPIC.rfind("/") + 1:]}',
            parameters=[
                {'base_topic': BASE_CAM_TOPIC},
                camera_processing_param_file
            ]
        ),

        Node(
            package='virtuoso_perception',
            executable='noise_filter',
            name=f'perception_noise_filter_{BASE_CAM_TOPIC[BASE_CAM_TOPIC.rfind("/") + 1:]}',
            parameters=[
                {'base_topic': BASE_CAM_TOPIC},
                camera_processing_param_file
            ]
        ),

        Node(
            package='virtuoso_perception',
            executable='buoy_cam_filter',
            parameters=[
                {'base_topic': BASE_CAM_TOPIC},
                buoy_param_file,
                {'filter_bounds': new_filter_bounds},
                # {'label_bounds': label_bounds},
                # {'buoy_border_px': buoy_border_px},
                # {'buoy_px_color_sample_size': buoy_px_color_sample_size},
                camera_processing_param_file
            ]
        ),

        Node(
            package='virtuoso_perception',
            executable='test_buoy_cam_filter',
            parameters=[
                {'base_topic': BASE_CAM_TOPIC}
            ]
        )
    ])
