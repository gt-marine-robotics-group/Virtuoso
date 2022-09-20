from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    sim_time = DeclareLaunchArgument('sim_time', default_value='False')
    sim_path = DeclareLaunchArgument('sim_path', default_value='')

    sim_time_config = LaunchConfiguration('sim_time', default='False')
    sim_path_config = LaunchConfiguration('sim_path', default='')

    processing = get_package_share_directory('virtuoso_processing')
    navigation = get_package_share_directory('virtuoso_navigation')
    localization = get_package_share_directory('virtuoso_localization')
    perception = get_package_share_directory('virtuoso_perception')
    controller = get_package_share_directory('virtuoso_controller')
    sensors = get_package_share_directory('virtuoso_sensors')

    return LaunchDescription([
        sim_time,
        sim_path,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sensors, 'launch', 'main.launch.py')),
            condition=UnlessCondition(sim_time_config)
        ),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(processing, 'launch', 'main.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization, 'launch', 'ekf.launch.py')),
            launch_arguments= {'sim_time': sim_time_config, 'sim_path': sim_path_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'main.launch.py')),
            launch_arguments={'sim_time': sim_time_config}.items()
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(perception, 'launch', 'find_buoys.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(controller, 'launch', 'main.launch.py')),
            launch_arguments={'sim_time': sim_time_config}.items()
        ),

        Node(
            package='virtuoso_autonomy',
            executable='robotX_gymkhana'
        )
    ])
