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

![euclidean_clustering](https://user-images.githubusercontent.com/59785089/153636767-fb965bd7-4![filtered_euclidean_clustering](https://user-images.githubusercontent.com/59785089/153638647-9f55caf0-3f58-438c-b15e-c432f330cd4a.png)
93a-4d6e-847a-ecf75f1bcbcd.png)

Next, the clusters found are filtered to only those which are buoys (by using information we have on each cluster such as position and height). Each buoy is also given a "value" property of 1.0 if tall and 0.5 if round.


