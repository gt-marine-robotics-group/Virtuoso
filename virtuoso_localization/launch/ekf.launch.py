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

    return LaunchDescription([
        usv_arg,
        Node(
            package='virtuoso_localization',
            executable='republisher'
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
