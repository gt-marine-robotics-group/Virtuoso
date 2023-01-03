from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    perception_share = get_package_share_directory('virtuoso_perception')
    
    usv_arg = DeclareLaunchArgument('usv')
    usv_config = LaunchConfiguration('usv')

    usv_config_str = None
    for arg in sys.argv:
        if arg.startswith('usv:='):
            usv_config_str = arg.split(':=')[1]

    ld = list()

    ld.append(usv_arg)

    if usv_config_str == 'robotx':
        ld.append(
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
            ),
        )
        ld.append(
            Node(
                package='virtuoso_sensors',
                executable='camera_republish'
            ),
        )
        ld.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='leftcam_to_baselink',
                arguments=['1.17', '-0.18', '1.245', '0', '0', '0', 'wamv/base_link', 'wamv/front_left_camera_link_optical'],
                condition=IfCondition(PythonExpression(["'", usv_config, "' == 'robotx'"]))
            )
        )
    else:
        with open(f'{perception_share}/config/{usv_config_str}/camera_config.yaml', 'r') \
            as stream:
            camera_data = yaml.safe_load(stream)

        for i, topic in enumerate(camera_data['camera_config']['all_camera_base_topics']):
            ld.append(
                Node(
                    package='usb_cam',
                    executable='usb_cam_node_exe',
                    name=f'usb_cam_{topic[topic.rfind("/") + 1:]}',
                    parameters=[
                        {'video_device': f'/dev/video{i}'}
                    ],
                    remappings=[
                        ('/image_raw', f'{topic}/image_raw'),
                        ('/camera_info', f'{topic}/empty_camera_info')
                    ]
                )
            )
        
        for i, frame in enumerate(camera_data['camera_config']['all_camera_frames']):
            ld.append(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=f'base_link_to_{frame}',
                    arguments=[
                        *(str(d) for d in camera_data['camera_config']['all_camera_transforms'][i]),
                        '0', '0', '0', 'base_link', frame
                    ]
                )
            )

    return LaunchDescription(ld)
