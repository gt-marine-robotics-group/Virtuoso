# Virtuoso Processing

## Contents
- [Autoware Auto Nodes](#autoware-auto-nodes)
  - [Ray Ground Classifier](#ray-ground-classifier)
- [Virtuoso Processing Nodes](#virtuoso-processing-nodes)
  - [lidar/self_filter_node.py](#lidarself\_filter\_nodepy)
  - [lidar/shore_filter_node.py](#lidarshore\_filter\_nodepy)
  - [camera/downscale_node.py](#cameradownscale\_nodepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [ray_ground_classifier.yaml](#ray_ground_classifieryaml)
  - [lidar_processing.yaml](#lidar_processingyaml)

## Autoware Auto Nodes

### Ray Ground Classifier
This node applies a ground filter to the incoming PointCloud. Points that are not the ground (in our case the water) are further processed.

## Virtuoso Processing Nodes

### lidar/self_filter_node.py
This node filters any points within a certain distance of the LIDAR. These points in most cases will be the USV itself.

### lidar/shore_filter_node.py
This node filters any points less a certain x-values and outside a certain range along the y-axis. This is not necessarily a "shore filter" as it was intending to be, but will filter anything outside of our desired field of view.

### camera/downscale_node.py
This node originally downscaled the raw image data, but now simply serves to republish the image. This is due to difficulties with perception on the downscaled image, but we hope to re-incorporate it in the future.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| usv/lidar_points | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | Used by the ground filter. |
| usv/camera_image | [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) | camera_link | Used by downscale_node. |
  
## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /processing/lidar/points_shore_filtered | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | Used by Nav2 to create costmap. |
| /processing/image/downscaled | [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) | camera_link | Used by perception package. |

## Parameters

### ray_ground_classifier.yaml
Key parameter here is sensor_height_m, as this will defintely change between USVs. Unfortunately, no documentation for the parameters found online.

### lidar_processing.yaml

| Node | Parameter | Type | Description |
|------|----------------|------|-------------|
| processing_self_filter | radius | float | The minimum distance from the LIDAR a point must be for it to be treated as not part of the robot. Any points within this radius are filtered. |
| processing_shore_filter | x_min | float | The minimum x-coordinate a point must have in order to not be filtered. For example, an x_min of 0 would filter all points behind the LIDAR. |
| processing_shore_filter | y_min | float | The minimum y-coordinate a point must have in order to not be filtered. For example, a y_min of 0 would filter all points to the right of the LIDAR. |
| processing_shore_filter | y_max | float | The maximum y-coordinate a point must have in order to not be filtered. For example, a y_max of 0 would filter all points to the left of the LIDAR. |
