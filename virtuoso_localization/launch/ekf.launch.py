from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import yaml
import sys

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_localization')

    sim_time_arg = DeclareLaunchArgument('sim_time', default_value='False')
    sim_time_config = LaunchConfiguration('sim_time', default='False')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]
    
    with open(pkg_share + '/config/' + usv_config_str + '/ekf_selection.yaml','r') as f:
        ekf_selector = yaml.safe_load(f)
    ekf_select = ekf_selector["ekf_launch"]["ekf_selection"]
    
    
    robot_localization_file_path = (pkg_share, '/config/', usv_config, ekf_select)
    

    return LaunchDescription([
        usv_arg,
        sim_time_arg,
        Node(
            package='virtuoso_localization',
            executable='republisher',
            parameters=[{
                'sim_time': sim_time_config
            }]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[robot_localization_file_path],
            remappings=[
            ("/odometry/filtered", "/localization/odometry"),
            ("/odometry/gps", "/odometry/gps2")
            ]
        ),    
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            respawn='true',
            remappings=[
            ('/imu', '/navsat/imu'),
            ('/gps/fix', '/navsat/gps'),
            ("/odometry/filtered", "/localization/odometry"),
            ("/odometry/gps", "/odometry/gps2")
            ],
            parameters=[            
                {"publish_filtered_gps": True},
                {"wait_for_datum": False},
                {"zero_altitude": False},
                {"yaw_offset": 0.0},
                {"use_odometry_yaw": True},
                {"delay": 0.0},
                {"frequency": 30.0},
                {"broadcast_utm_transform": True},
            ]
        )
    ])
