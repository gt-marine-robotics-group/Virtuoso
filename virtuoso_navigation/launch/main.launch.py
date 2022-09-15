from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    sim_time = DeclareLaunchArgument('sim_time')
    sim_time_config = LaunchConfiguration('sim_time', default='False')

    pkg_share = get_package_share_directory('virtuoso_navigation')

    # the launch file which we pass stvl in as a plugin an the params file
    bringup_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    rviz_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')
    nav2_params_file = os.path.join(pkg_share, 'param', 'nav2.param.yaml')

    return LaunchDescription([
        sim_time,
        DeclareLaunchArgument(
            name='sim_time',
            default_value='False'
        ),
        Node(
            package='virtuoso_navigation',
            executable='set_goal'
        ),
        Node(
            package='virtuoso_navigation',
            executable='waypoints'
        ),
        Node(
            package='virtuoso_navigation',
            executable='choose_PID'
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_launch_file),launch_arguments={'params_file': nav2_params_file,
        'use_sim_time': sim_time_config}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz_launch_file)),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', arguments=[nav2_params_file],
        parameters=[{'use_sim_time': sim_time_config}]),

        # Currently, state estimation only using odom frame for localization, so no difference between 
        # odom and map frame. Transformation being used for the costmaps.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map']
        )
    ])
