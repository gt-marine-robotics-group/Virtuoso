# Virtuoso Processing

## Building the package

1. Have the Virtuoso repo cloned and ROS2 set up
2. Have [Skimage](https://scikit-image.org/docs/dev/install.html), numpy, cv_bridge installed
   - Skimage is a package containing different image processing alogrithms. In our pipeline, we use `rgb2gray()` for grayscaling and `downscale_local_mean()` for    downscaling camera data.
3. Install [Autoware.auto](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-no-ade.html) and add to workspace
   - Autoware.auto is a collection of packages for self-driving vehicles. In our pipeline, we use `ray_ground_classifier_nodes` for downscaling lidar data.
4. Install [STVL](https://navigation.ros.org/tutorials/docs/navigation2_with_stvl.html) and add to workspace
   - [Spatio-Temporal Voxel Layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) is a [Nav2](https://github.com/ros-planning/navigation2) costmap plugin. It downsamples PointCloud data to generate a voxel grid and a costmap. In our pipeline, we use it for downsampling lidar data, and the costmap         generated will be useful for the navigation pipeline.
5. From workspace, run `colcon build --packages-up-to virtuoso_processing`

## Running the package
There are three launch files:
1. [lidar_processing.launch.py](launch/lidar_processing.launch.py)
2. [camera_processing.launch.py](launch/camera_processing.launch.py)
3. <span>[main.launch.py](launch/main.launch.py)</span>
   - Running this will launch the other two

To run the package, simply run from the Virtuoso directory `ros2 launch virtuoso_processing main.launch.py`.

## LiDAR Processing
LiDAR processing is done through the [lidar_processing.launch.py](launch/lidar_processing.launch.py) launch file.

![processing_diagram](https://user-images.githubusercontent.com/59785089/151290168-cad0bafd-5d35-425c-b0ff-2a8ea4655b64.png)

Pointcloud data is published by the wamv to the topic `wamv/sensors/lidars/lidar_wamv/points`.

![raw_wamv](https://user-images.githubusercontent.com/59785089/151288813-222ff8cc-246b-4fbe-9512-c0a1371fd271.png)


The raw PointCloud data is first processed by the [ray_ground_classifier_nodes](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/ray-ground-classifier-nodes-design.html) which publish the non-ground points to the topic `points_nonground`.

![points_nonground](https://user-images.githubusercontent.com/59785089/151288846-e46892e1-b2e4-430a-8a31-af9cda552565.png)

Next, the data passes through a self_filter which removes pointcloud data from the body of the robot. The filtered data is published to `points_self_filtered`.

![points_self_filtered](https://user-images.githubusercontent.com/59785089/151288988-007b2380-a45f-4274-af76-ee866ce8e579.png)

## Camera Processing
Camera processing is done through the [camera_processing.launch.py](launch/camera_processing.launch.py) launch file.

![camera_pipeline](https://user-images.githubusercontent.com/59785089/145681124-95e74a68-2d8a-4194-b5a2-4f8fc6396d0c.png)

The raw Camera data is first processed by the [grayscale](virtuoso_processing/grayscale.py) node which publishes the grayscaled camera data (in a mono8 encoding) to the topic `grayscaled_image`. You can visualize this using rqt_image_view:

![grayscaled_image](https://user-images.githubusercontent.com/59785089/142947578-47bafa01-45ec-4d84-939b-36713a3d3e6f.png)

Then, the Camera data is processed by the [downscale](virtuoso_processing/downscale.py) node which publishes the downscaled data to the topic `downscaled_image`. Again, you can visualize this with rqt_image_view:

![downscaled_image](https://user-images.githubusercontent.com/59785089/142947626-a01bd3bf-7932-4168-a4fa-2f34ef29a5d1.png)

The Rosbag with Camera data used to test the pipeline can be found [here](https://drive.google.com/file/d/0B7x5e7bDeXqpeFhQd3FPdVdkTzQ/view?resourcekey=0-eNpU0y4ISgyqhlsBSOOJ8w).

