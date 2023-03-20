from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_auxiliary')

    usv_arg = DeclareLaunchArgument('usv')

    usv_config = LaunchConfiguration('usv')

    ball_shooter_params = (pkg_share, '/config/', usv_config, '/ball_shooter.yaml')

    return LaunchDescription([
        usv_arg,

        Node(
            package='virtuoso_auxiliary',
            executable='ball_shooter',
            parameters=[ball_shooter_params]
        )
    ])