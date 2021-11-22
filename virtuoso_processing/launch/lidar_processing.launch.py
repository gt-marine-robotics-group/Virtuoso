from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# Using ray_ground_classifier and voxel_grid_node from Autoware.auto

def generate_launch_description(): 

    ray_ground_classifier_param_file = 'virtuoso_processing/param/ray_ground_classifier.param.yaml'

    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )

    voxel_grid_node_param_file = 'virtuoso_processing/param/voxel_grid_node.param.yaml'

    voxel_grid_node_param = DeclareLaunchArgument(
        'voxel_grid_node_param_file',
        default_value=voxel_grid_node_param_file,
        description='Path to config file for Voxel Grid Node'
    )


    return LaunchDescription([
        ray_ground_classifier_param,
        voxel_grid_node_param,
        Node(
            package='ray_ground_classifier_nodes',
            executable='ray_ground_classifier_cloud_node_exe',
            parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
            remappings=[("points_in", "/horizontal_laser_3d")] # points_in comes from raw LIDAR data
        ),
        Node(
            package='voxel_grid_nodes',
            executable='voxel_grid_node_exe',
            parameters=[LaunchConfiguration('voxel_grid_node_param_file')],
            remappings=[
                ("points_in", "points_nonground"), # points_in come from topic published to by ray_ground_classifier_nodes 
                ("points_downsampled", "points_fused_downsampled")
            ]
        ) 
    ])