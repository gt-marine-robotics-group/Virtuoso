from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
import sys
import yaml

def generate_launch_description():

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    pkg_share = get_package_share_directory('virtuoso_navigation')

    bringup_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    rviz_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')
    nav2_params_file = (pkg_share, '/config/', usv_config, '/nav2.yaml')
    waypoints_param_file = (pkg_share, '/config/', usv_config, '/waypoints.yaml')
    rotate_param_file = (pkg_share, '/config/', usv_config, '/rotate.yaml')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]

    is_sim_time = bool('vrx' in usv_config_str)

    waypoint_data = None
    with open(f'{pkg_share}/config/{usv_config_str}/waypoints.yaml', 'r') as stream:
        waypoint_data = yaml.safe_load(stream)

    use_nav2 = waypoint_data['navigation_waypoints']['ros__parameters']['use_nav2']

    ld = [
        usv_arg,

        Node(
            package='virtuoso_navigation',
            executable='waypoints',
            parameters=[waypoints_param_file]
        ),
        Node(
            package='virtuoso_navigation',
            executable='translate'
        ),
        Node(
            package='virtuoso_navigation',
            executable='station_keeping'
        ),
        Node(
            package='virtuoso_navigation',
            executable='rotate',
            parameters=[rotate_param_file]
        ),
        Node(
            package='virtuoso_navigation',
            executable='approach_target'
        ),

        # Currently, state estimation only using odom frame for localization, so no difference between 
        # odom and map frame. Transformation being used for the costmaps.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    ]

    if is_sim_time:
        ld.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_links',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'wamv']
            )
        )

    if use_nav2:
        ld.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_launch_file),launch_arguments={'params_file': nav2_params_file,
        'use_sim_time': is_sim_time}.items()))
        ld.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_file),
            condition=is_sim_time
        ))
    elif is_sim_time:
        ld.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rviz.launch.py'))
            )
        )

    return LaunchDescription(ld)
