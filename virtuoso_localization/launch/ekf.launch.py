import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    sim_path = DeclareLaunchArgument('sim_path', default_value='')
    sim_time = DeclareLaunchArgument('sim_time', default_value='False')

    # pkg_share = FindPackageShare(package='virtuoso_localization').find('virtuoso_localization')
    # robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 
    pkg_share = get_package_share_directory('virtuoso_localization')
    robot_localization_file_path = (pkg_share, '/config/', LaunchConfiguration('sim_path', default=''), 'ekf.yaml')

    return LaunchDescription([
        sim_path,
        sim_time,
        Node(
            package='virtuoso_localization',
            executable='continual_ekf',
            parameters=[{
                'sim_time': LaunchConfiguration('sim_time', default='False')
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
            ("/imu", "/navsat/imu"),
            ("/gps/fix", "/navsat/gps"),
            ("/odometry/filtered", "/localization/odometry"),
            ("/odometry/gps", "/odometry/gps2")
        ],
        #output='screen',
        parameters=[            
            {"publish_filtered_gps": True},
            {"wait_for_datum": False},
            {"zero_altitude": False},
            {"yaw_offset": 0.0},
            {"use_odometry_yaw": True},
            #{"magnetic_declination_radians": 0.33929201},
            {"magnetic_declination_radians": 0.0},
            {"delay": 0.0},
            {"frequency": 30.0},
            {"broadcast_utm_transform": True}]
        )
    ])
