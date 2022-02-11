# Virtuoso Perception

## Building the Package

This package depends on `virtuoso_processing` and `euclidean_cluster_nodes`. Follow the documentation for building [`virtuoso_processing`](../virtuoso_processing) to ensure this package will build. Make sure [Autoware.auto](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-no-ade.html) is installed and sourced as well to build the `euclidean_cluster_node`.

To build, run `colcon build --packages-up-to virtuoso_perception`.

## Running the Package

There are three launch file:
1. [euclidean_clustering.launch.py](launch/euclidean_clustering.launch.py)
   - Launches the `euclidean_cluster_nodes` by autoware.auto
3. [find_buoys.launch.py](launch/find_buoys.launch.py)
   - Launches the previous launch file and a node which publishes the found buoys
5. [find_and_classify_buoys.launch.py](launch/find_and_classify_buoys.launch.py)
   - Launches the previous launch file and a node which classifies the buoys by color

For [task 3](https://github.com/osrf/vrx/wiki/vrx_2022-perception_task), we will need to launch `find_and_classify_buoys.launch.py`.

## Using Lidar to Find Buoys

Finding the buoys is accomplished through the [find_buoys.launch.py](launch/find_buoys.launch.py) launch file.

![lidar_buoys](https://user-images.githubusercontent.com/59785089/153636183-d05ee8a9-2788-458e-b0b2-a01492a4672b.svg)

PointCloud data already processed by [`virtuoso_processing`](../virtuoso_processing) is used by the euclidean clustering node to identify obstacles.

![euclidean_clustering](https://user-images.githubusercontent.com/59785089/153638944-90cf0f44-b5af-4365-9b90-bd802b1005f7.png)

Next, the clusters found are filtered to only those which are buoys by using information we have on each cluster such as position and height). Each buoy is also given a "value" property of 1.0 if tall and 0.5 if round.

![filtered_euclidean_clustering](https://user-images.githubusercontent.com/59785089/153638720-3d757d1b-1fa3-4c41-8967-24b705a45bc6.png)

## Using Camera to Classify Buoys

Identifying the buoys (and then classifying) is done through the [find_and_classify_buoys.launch.py](launch/find_and_classify_buoys.launch.py) launch file.

![classify_buoys drawio](https://user-images.githubusercontent.com/59785089/153639863-7c7aa278-b559-47b0-be8e-b5c0862db60f.svg)

First, the node takes in raw camera data.

![img_raw](https://user-images.githubusercontent.com/59785089/153639997-637d9401-55fe-4ee4-bcc3-7156a1e2dbfd.png)

Then, the node filters the image by black, white, red/orange, and green. Below, you can see an example of what an image filtered for red/orange looks like.

![filter_color](https://user-images.githubusercontent.com/59785089/153640200-860e51c7-fdfd-4670-b48e-c5764e7a5bec.png)

Finally, for each filtered image, we use cv2 to find contours. Any contours which have an area > 1000 are classified as a buoy. The buoy is added to an array of detected buoys. Each classified buoy is matched up the corresponding box found by euclidean clustering. The boxes are given an arbitrary "value" property of 0 to 5 depending on what type of buoy it is (tall black, tall green, round black, etc).

