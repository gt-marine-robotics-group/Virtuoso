from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('virtuoso_autonomy')

    sim_time = DeclareLaunchArgument('sim_time', default_value='False')
    usv = DeclareLaunchArgument('usv')

    sim_time_config = LaunchConfiguration('sim_time', default='False')
    usv_config = LaunchConfiguration('usv')

    processing = get_package_share_directory('virtuoso_processing')
    navigation = get_package_share_directory('virtuoso_navigation')
    localization = get_package_share_directory('virtuoso_localization')
    perception = get_package_share_directory('virtuoso_perception')
    controller = get_package_share_directory('virtuoso_controller')
    sensors = get_package_share_directory('virtuoso_sensors')

    docking_config_file = (pkg_share, '/config/', usv_config, '/docking.yaml')

    return LaunchDescription([
        sim_time,
        usv,

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(sensors, 'launch', 'main.launch.py')),
        #     condition=UnlessCondition(sim_time_config)
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(processing, 'launch', 'main.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization, 'launch', 'ekf.launch.py')),
            launch_arguments= {'sim_time': sim_time_config, 'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'main.launch.py')),
            launch_arguments={'sim_time': sim_time_config, 'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(perception, 'launch', 'find_docks.launch.py')),
            launch_arguments={'usv': usv_config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(controller, 'launch', 'main.launch.py')),
            launch_arguments={'sim_time': sim_time_config, 'usv': usv_config}.items()
        ),

        Node(
            package='virtuoso_autonomy',
            executable='robotX_docking',
            parameters=[docking_config_file]
        )
    ])