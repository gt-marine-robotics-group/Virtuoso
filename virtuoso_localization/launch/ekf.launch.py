from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_localization')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    robot_localization_file_path = (pkg_share, '/config/', usv_config, '/ekf.yaml')
    sensor_config_file_path = (pkg_share, '/config/', usv_config, '/sensor_config.yaml')

    return LaunchDescription([
        usv_arg,
        Node(
            package='virtuoso_localization',
            executable='republisher',
            parameters=[sensor_config_file_path]
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
            respawn=True,
            remappings=[
            ('/imu', '/navsat/imu'),
            ('/gps/fix', '/navsat/gps'),
            ("/odometry/filtered", "/localization/odometry"),
            ("/odometry/gps", "/odometry/gps2")
            ],
            parameters=[            
                robot_localization_file_path
            ]
        )
    ])
