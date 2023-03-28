from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_autonomy')

    usv_arg = DeclareLaunchArgument('usv')

    usv_config = LaunchConfiguration('usv')

    finals_config_file = (pkg_share, '/config/', usv_config, '/finals.yaml')

    return LaunchDescription([
        usv_arg, 

        Node(
            package='virtuoso_autonomy',
            executable='roboboat_finals',
            parameters=[finals_config_file]
        )
    ])