from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():

    sensors = get_package_share_directory('virtuoso_sensors')

    lidar_config = os.path.join(sensors, 'config', 'vlp16.yml')

    pointcloud_converter_config = os.path.join(sensors, 'config', 'vlp16-convert.yml') 
    with open(pointcloud_converter_config, 'r') as f:
        pointcloud_converter_params = yaml.safe_load(f)['velodyne_convert_node']['ros_parameters']
    pointcloud_converter_params['calibration'] = os.path.join(sensors, 'params', 'vlp16db.yml')

    velodyne_node = Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            parameters=[lidar_config]
        )

    # pointcloud_converter_node = Node(
    #     package='velodyne_pointcloud'
    # )

    return LaunchDescription([
        velodyne_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=velodyne_node,
                on_exit=[EmitEvent(
                    event=Shutdown()
                )]
            )
        )
    ])