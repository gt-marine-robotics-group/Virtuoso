# Virtuoso Processing

## Building the package

1. Have the Virtuoso repo cloned and ROS set up
2. Have [Skimage](https://scikit-image.org/docs/dev/install.html), numpy, cv_bridge installed
3. Install [Autoware.auto](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-no-ade.html) and add to workspace
4. Insall [STVL](https://navigation.ros.org/tutorials/docs/navigation2_with_stvl.html) and add to workspace
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

![lidar_pipeline](https://user-images.githubusercontent.com/59785089/145680590-86fdb615-58e0-4c28-abd0-dc7d82acda39.png)

The raw PointCloud data is first processed by the [ray_ground_classifier_nodes](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/ray-ground-classifier-nodes-design.html) which publish the non-ground points to the topic `points_nonground`. You can visualize this data with Rviz:

![points_nonground](https://user-images.githubusercontent.com/59785089/145680125-80deb730-46f1-4c49-807c-bfd4417d8df8.png)

Here, you can see the raw data in black and white and the `points_nonground` in color:

![points_nonground_with_raw](https://user-images.githubusercontent.com/59785089/145680192-d3175247-5baf-48ad-9e15-f06f3c369dd2.png)

Then, the PointCloud data is run through STVL which downsamples the data and also generates a costmap. The downsampled data is published to `local_costmap/voxel_grid`, and the costmap is published to `local_costmap/costmap`. On Rviz, you can visualize the downsampled data with the costmap:

![stvl](https://user-images.githubusercontent.com/59785089/145680382-e4059c50-2791-49df-b338-84d2e59608d3.png)

The Rosbag with PointCloud data used to test the pipeline can be found [here](https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-02-09-13-17-39.bag).

## Camera Processing
Camera processing is done through the [camera_processing.launch.py](launch/camera_processing.launch.py) launch file.

![camera_pipeline](https://user-images.githubusercontent.com/59785089/145681124-95e74a68-2d8a-4194-b5a2-4f8fc6396d0c.png)

The raw Camera data is first processed by the [grayscale](virtuoso_processing/grayscale.py) node which publishes the grayscaled camera data (in a mono8 encoding) to the topic `grayscaled_image`. You can visualize this using rqt_image_view:

![grayscaled_image](https://user-images.githubusercontent.com/59785089/142947578-47bafa01-45ec-4d84-939b-36713a3d3e6f.png)

Then, the Camera data is processed by the [downscale](virtuoso_processing/downscale.py) node which publishes the downscaled data to the topic `downscaled_image`. Again, you can visualize this with rqt_image_view:

![downscaled_image](https://user-images.githubusercontent.com/59785089/142947626-a01bd3bf-7932-4168-a4fa-2f34ef29a5d1.png)

The Rosbag with Camera data used to test the pipeline can be found [here](https://drive.google.com/file/d/0B7x5e7bDeXqpeFhQd3FPdVdkTzQ/view?resourcekey=0-eNpU0y4ISgyqhlsBSOOJ8w).

