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
