from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_sensors')
    velodyne_share = get_package_share_directory('velodyne')

    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    vlp16_config_file = os.path.join(pkg_share, 'config', 'vlp16.yml')

    return LaunchDescription([
        usv_arg,

        DeclareLaunchArgument(
            'vlp16_node_param_file',
            default_value=vlp16_config_file
        ),
        # Node(
        #     package='velodyne_nodes',
        #     namespace='lidar_front',
        #     executable='velodyne_cloud_node_exe',
        #     parameters=[LaunchConfiguration('vlp16_node_param_file')],
        #     remappings=[("topic", "points_xyzi")],
        #     arguments=["--model", "vlp16"]
        # ),
        # Node(
        #     package='virtuoso_sensors',
        #     executable='lidar_republish'
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(velodyne_share, 'launch', 'velodyne-all-nodes-VLP16-launch.py')
            )
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_to_baselink',
            arguments=['0', '0', '1.245', '0', '0', '0', 'wamv/base_link', 'wamv/lidar_wamv_link'],
            condition=IfCondition(PythonExpression(["'", usv_config, "' == 'robotx'"]))
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidarfront_to_wamvlidar',
            # arguments=['0', '0', '0', '0', '-3.14159', '0', 'wamv/lidar_wamv_link', 'lidar_front']
            arguments=['0', '0', '0', '0', '0', '0', 'wamv/lidar_wamv_link', 'velodyne'],
            # arguments=['0', '0', '0', '0', '0', '3.14159', 'wamv/base_link', 'wamv/lidar_wamv_link']
            # arguments=['0', '0', '0', '3.14149', '0', '0', 'wamv/base_link', 'wamv/lidar_wamv_link']
            condition=IfCondition(PythonExpression(["'", usv_config, "' == 'robotx'"]))
        )
    ])