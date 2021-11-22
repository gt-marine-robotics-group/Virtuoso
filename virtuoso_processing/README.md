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


Then, the PointCloud data is processed by the [voxel_grid_nodes](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/voxel-grid-nodes-design.html) which publish the downsampled data to the topic `points_fused_downsampled`. Again, you can visualize this with Rviz:

The Rosbag with PointCloud data used to test the pipeline can be found [here](https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-15-51-36.bag).

## Camera Processing
Camera processing is done through the [camera_processing.launch.py](launch/camera_processing.launch.py) launch file.

The raw Camera data is first processed by the [grayscale](virtuoso_processing/grayscale.py) node which publishes the grayscaled camera data (in a mono8 encoding) to the topic `grayscaled_image`. You can visualize this using rqt_image_view:

Then, the Camera data is processed by the [downscale](virtuoso_processing/downscale.py) node which publishes the downscaled data to the topic `downscaled_image`. Again, you can visualize this with rqt_image_view:

The Rosbag with Camera data used to test the pipeline can be found [here](https://drive.google.com/file/d/0B7x5e7bDeXqpeFhQd3FPdVdkTzQ/view?resourcekey=0-eNpU0y4ISgyqhlsBSOOJ8w).

