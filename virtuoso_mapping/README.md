# Virtuoso Mapping

## Contents
- [Virtuoso Nodes](#virtuoso-nodes)
    - [occupancy_map_generator_node.py](#occupancy\_map\_generator\_nodepy)
    - [pcl_to_map_frame_node.cpp](#pcl\_to\_map\_frame\_nodecpp)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
    - [occupancy_map_generator.yaml](#occupancy\_map\_generatoryaml)

## Virtuoso Nodes

### occupancy_map_generator_node.py
This node generates an occupancy grid using voxels created from the raw lidar data.

### pcl_to_map_frame_node.cpp
This node transforms the Lidar voxels created in `virtuoso_perception` from the lidar frame to the map frame. The transformed voxels are then used by the occupancy map generator.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /perception/lidar/voxels | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | After transformed to map frame, used to populate occupancy grid. | 

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /mapping/occupancy_map | [nav_msgs/OccupancyGrid](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html) | map | Occupancy map of obstacles detected by the lidar. |

## Parameters

### occupancy_map_generator.yaml

Note: it is recommeded to make the map a square (i.e. width and height equal), as there may be some undetected bugs otherwise...

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| occupancy_map_generator | resolution | float | The resolution of the map (i.e. 0.1, 0.5, 1 meter wide squares). |
| occupancy_map_generator | grid_width | int | The width in meters of the occupancy map. |
| occupancy_map_generator | grid_height | int | The height in meters of the occupancy map. |
| occupancy_map_generator | update_delay | float | Delay in seconds to wait before updating the map. |
| occupancy_map_generator | point_weight | int | Mitchell please add. |
| occupancy_map_generator | decay_rate | int | Mitchell please add. |