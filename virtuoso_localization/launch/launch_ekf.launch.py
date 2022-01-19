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
        #Node(
         #   package='virtuoso_localization',
          #  executable='test_pub',
        #),

 
        #Node(
    #package = "tf2_ros", 
    #executable = "static_transform_publisher",
    #name="gps_wamv_link_broadcaster",
    #arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "wamv/gps_wamv_link"]
     #),
       # Node(
    #package = "tf2_ros", 
    #executable = "static_transform_publisher",
    #name="gps_wamv_link_broadcaster2",
    #arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "wamv/gps_wamv_link"]
    # ),
     #Node(
   # package = "tf2_ros", 
    #executable = "static_transform_publisher",
    #name="imu_wamv_link_broadcaster",
    #arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "wamv/imu_wamv_link"]
    # ),
     #Node(
    #package = "tf2_ros", 
    #executable = "static_transform_publisher",
    #name="map_link_broadcaster",
    #arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"]
     #),
     Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    #output='screen',
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
         #{"background_b": 200},
         {"publish_filtered_gps": True},
         {"wait_for_datum": False},
         {"zero_altitude": False},
         {"yaw_offset": 0.0},
         {"use_odometry_yaw": True},
         {"magnetic_declination_radians": 0.33929201},
         {"delay": 0.0},
         {"frequency": 30.0},
         {"broadcast_utm_transform": True}]
        )
    ])
