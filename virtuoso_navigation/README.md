# Virtuoso Navigation

## Building the Package

This package depends on `virtuoso_processing`, `virtuoso_localization`, and `spatio_temporal_voxel_layer`. 

To build, run `colcon build --packages-up-to virtuoso_navigation`.

## Running the Package

Simply run the launch file: `ros2 launch virtuoso_navigation main.launch.py`.

Note: The filtered Lidar comes from `virtuoso_processing`, so that will need to be running. The transormation between the lidar frame and odom is done by `virtuoso_localization`, so that will need to be running for the global costmap to be accurate.

## Generating Costmap

A local costmap, using `spatio_temporal_voxel_layer` to generate a voxel grid, is created from the filtered PointCloud2 data. It is published to `/local_costmap/costmap`.

![local_costmap](https://user-images.githubusercontent.com/59785089/151289187-5a7f69e8-9790-4889-bada-f9a9331c9e94.png)

The global costmap is also created from the processed data and localization data. It is published to `/global_costmap/costmap`.

![global_costmap](https://user-images.githubusercontent.com/59785089/151289292-ea8ddd43-1586-4462-bc2a-6417429f62ec.png)

## Navigation

### /set_goal
The SetGoal node takes in a desired goal (published to `/virtuoso_navigation/set_goal`). 
The goal is passed on to nav2 which generates a [Path](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Path.msg) published to `/plan`.
Nav2 also publishes a velocity to `/cmd_vel` that will be used by our motors.
