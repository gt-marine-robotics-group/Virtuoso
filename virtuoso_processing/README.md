# Virtuoso Processing

## Building the package

1. Have the Virtuoso repo cloned and ROS set up
2. Install [Skimage](https://scikit-image.org/docs/dev/install.html)
3. Install [Autoware.auto](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-no-ade.html)
4. From the AutowareAuto directory, run `colcon build --packages-up-to ray_ground_classifier_nodes` and `colcon build --packages-up-to voxel_grid_nodes`
5. Run `source install/setup.bash`
6. `cd` to the Virtuoso directory
7. Run `colcon build`

## Running the package
There are three launch files:
1. lidar_processing.launch.py
2. camera_processing.launch.py
3. <span>main.launch.py</span>
   - Running this will launch the other two

To run the package, simply run from the Virtuoso directory `ros2 launch virtuoso_processing main.launch.py`.

## LiDAR Processing
LiDAR processing is done through the [lidar_processing.launch.py](launch/lidar_processing.launch.py) launch file.

The raw PointCloud data is first processed by the [ray_ground_classifier_nodes](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/ray-ground-classifier-nodes-design.html) which publish the non-ground points to the topic `points_nonground`. You can visualize this data with Rviz:

![points_nonground](https://user-images.githubusercontent.com/59785089/142947445-346a92d1-5243-4bd3-ad09-d5725a31c82c.png)

Then, the PointCloud data is processed by the [voxel_grid_nodes](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/voxel-grid-nodes-design.html) which publish the downsampled data to the topic `points_fused_downsampled`. Again, you can visualize this with Rviz:

![points_fused_downsample](https://user-images.githubusercontent.com/59785089/142947525-302fa4a5-84fb-4fca-887d-64d174ce0128.png)

The Rosbag with PointCloud data used to test the pipeline can be found [here](https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-15-51-36.bag).

## Camera Processing
Camera processing is done through the [camera_processing.launch.py](launch/camera_processing.launch.py) launch file.

The raw Camera data is first processed by the [grayscale](virtuoso_processing/grayscale.py) node which publishes the grayscaled camera data (in a mono8 encoding) to the topic `grayscaled_image`. You can visualize this using rqt_image_view:

![grayscaled_image](https://user-images.githubusercontent.com/59785089/142947578-47bafa01-45ec-4d84-939b-36713a3d3e6f.png)

Then, the Camera data is processed by the [downscale](virtuoso_processing/downscale.py) node which publishes the downscaled data to the topic `downscaled_image`. Again, you can visualize this with rqt_image_view:

![downscaled_image](https://user-images.githubusercontent.com/59785089/142947626-a01bd3bf-7932-4168-a4fa-2f34ef29a5d1.png)

The Rosbag with Camera data used to test the pipeline can be found [here](https://drive.google.com/file/d/0B7x5e7bDeXqpeFhQd3FPdVdkTzQ/view?resourcekey=0-eNpU0y4ISgyqhlsBSOOJ8w).

