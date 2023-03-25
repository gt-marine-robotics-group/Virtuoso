from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import sys
import os
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_sensors')
    perception_share = get_package_share_directory('virtuoso_perception')
    velodyne_share = get_package_share_directory('velodyne')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]

    lidars = None
    with open(f'{perception_share}/config/{usv_config_str}/lidar_config.yaml', 'r') as stream:
        lidars = yaml.safe_load(stream)

    urg_params_file = (pkg_share, '/config/urg.yml')

    # return LaunchDescription([
    #     usv_arg,

        # Node(
        #     package='urg_node',
        #     executable='urg_node_driver',
        #     parameters=[urg_params_file]
        # ),
        # Node(
        #     package='virtuoso_sensors',
        #     executable='laser_to_pcd'
        # )

    # ])

    ld = list()

    ld.append(usv_arg)

    for i in range(len(lidars['lidar_config']['all_lidar_types'])):
        if lidars['lidar_config']['all_lidar_types'] == 'vlp16':
            ld.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(velodyne_share, 'launch', 'velodyne-all-nodes-VLP16-launch.py')
                )
            ))
            ld.append(Node(
                package='virtuoso_sensors',
                executable='lidar_republish'
            ))
        else:
            ld.append(Node(
                package='urg_node',
                executable='urg_node_driver',
                parameters=[urg_params_file, {'serial_port': f'/dev/ttyACM{i + 0}'}],
                remappings=[('/scan', f'{lidars["lidar_config"]["all_lidar_base_topics"][i]}/laser')]
            ))
            ld.append(Node(
                package='virtuoso_sensors',
                executable='laser_to_pcd',
                remappings=[
                    ('/scan', f'{lidars["lidar_config"]["all_lidar_base_topics"][i]}/laser'),
                    ('/output', f'{lidars["lidar_config"]["all_lidar_base_topics"][i]}/points')
                ],
                parameters=[{'frame': lidars['lidar_config']['all_lidar_frames'][i]}]
            ))
            ld.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'base_link_to_{lidars["lidar_config"]["all_lidar_frames"][i]}',
                arguments=[
                    *(str(d) for d in lidars['lidar_config']['all_lidar_transforms'][i]),
                    '0.0', '0', '0', 'base_link', lidars['lidar_config']['all_lidar_frames'][i]
                ]
            ))
    
    return LaunchDescription(ld)
