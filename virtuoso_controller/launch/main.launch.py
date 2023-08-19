from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_controller')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    pid_params_file = (pkg_share, '/config/', usv_config, '/pidgains.yaml')
    cmd_generator_params_file = (pkg_share, '/config/', usv_config, 
        '/cmd_generator.yaml')

    return LaunchDescription([
        usv_arg,

        Node(
            package='virtuoso_controller',
            executable='basic_pid',
            parameters = [pid_params_file]
        ),
        Node(
            package='virtuoso_controller',
            executable='velocity_pid',
            parameters = [pid_params_file]
        ),
        Node(
            package='virtuoso_controller',
            executable='motor_cmd_generator',
            parameters=[cmd_generator_params_file]
        ),
        Node(
            package='virtuoso_controller',
            executable='cmd_vel_generator'
       ),
        Node(
            package='virtuoso_controller',
            executable='choose_pid'
        ),
    ])
