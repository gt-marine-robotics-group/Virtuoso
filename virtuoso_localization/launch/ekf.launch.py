import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

pkg_share = FindPackageShare(package='virtuoso_localization').find('virtuoso_localization')
robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='virtuoso_localization',
            executable='continual_ekf',
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
    output='screen',
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
