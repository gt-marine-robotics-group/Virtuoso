import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_localization')
    pkg_share2 = get_package_share_directory('virtuoso_controller')    
    pkg_share3 = get_package_share_directory('virtuoso_navigation') 
        
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/ekf.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_share2, 'launch/pid_vel.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_share3, 'launch/main.launch.py'))),
        Node(
            package='virtuoso_autonomy',
            executable='getTask1Goal',
        ),
       Node(
            package='virtuoso_autonomy',
            executable='test_waypoint_generator',
        ),
    ])
