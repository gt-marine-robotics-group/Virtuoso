# Virtuoso Perception

## Contents
- [Autoware Auto Nodes](#autoware-auto-nodes)
  - [Ray Ground Classifier](#ray-ground-classifier)
  - [Euclidean Clustering](#euclidean-clustering)
  - [Voxel Grid](#voxel-grid)
- [Virtuoso Perception Nodes](#virtuoso-perception-nodes)
  - [lidar_processing/self_filter_node.py](#lidar\_processingself\_filter\_nodepy)
  - [lidar_processing/shore_filter_node.py](#lidar\_processingshore\_filter\_nodepy)
  - [camera_processing/resize_node.py](#camera\_processingresize\_nodepy)
  - [camera_processing/noise_filter_node.py](#camera\_processingnoise\_filter\_nodepy)
  - [buoys/find_buoys_node.py](#buoysfind\_buoys\_nodepy)
  - [buoys/buoy_filter_node.py](#buoysbuoy\_filter\_nodepy)
  - [buoys/channel_node.py](#buoyschannel\_nodepy)
  - [code/scan_code_node.py](#codescan\_code\_nodepy)
  - [dock/find_dock_codes_node.py](#dockfind\_dock\_codes\_nodepy)
  - [dock/find_dock_entrances_node.py](#dockfind\_dock\_entrances\_nodepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [External Services](#external-services)
- [Parameters](#parameters)
  - [ray_ground_classifier.yaml](#ray\_ground\_classifieryaml)
  - [lidar_processing.yaml](#lidar\_processingyaml)
  - [camera_processing.yaml](#camera\_processingyaml)
  - [euclidean_clustering.yaml](#euclidean\_clusteringyaml)
  - [voxel_grid_node.yaml](#voxel\_grid\_nodeyaml)
  - [buoys.yaml](#buoysyaml)
  - [code.yaml](#codeyaml)
  - [dock.yaml](#dockyaml)

## Autoware Auto Nodes

### Ray Ground Classifier
This node applies a ground filter to the incoming PointCloud. Points that are not the ground (in our case the water) are further processed.

### Euclidean Clustering
This node finds clusters from the voxel grid created by Nav2. These clusters can then be interpreted as objects like buoys or docks.

### Voxel Grid
This node takes in the voxel grid created by Nav2 and creates voxels of those voxels. These are used to help identify the entrances to docks as, if tuned correctly, there can be about 1 voxel per entry point.

## Virtuoso Perception Nodes

### lidar_processing/self_filter_node.py
This node filters any points within a certain distance of the LIDAR. These points in most cases will be the USV itself.

### lidar_processing/shore_filter_node.py
This node filters any points less a certain x-values and outside a certain range along the y-axis. This is not necessarily a "shore filter" as it was intending to be, but will filter anything outside of our desired field of view.

### camera_processing/noise_filter_node.py
This node removes noise from an image. Because our environment tends to be quite sparse (water with fairly distinct objects on them like buoys), we prefer "over-filtering" noise from an image than not removing enough. Losing small details from incoming image data is not a big deal.

### camera_processing/resize_node.py
This node resizes the incoming image so as to make image computations further down the stack less computationally expensive.

### buoys/find_buoys_node.py
This node uses the euclidean clusters to determine the location of buoys (Lidar). Buoys are assigned a value corresponding to their height. Buoys with a value >= 1 are considered "tall" buoys.

### buoys/buoy_filter_node.py
This node takes processed image data and filters out objects that are not one of the buoy types specified. It returns the contours of these buoys.

### buoys/channel_node.py
This node, when requested, will activate the image processing for the front 2 cameras on the USV and return 2 null points. Any requests then sent while there is procesed image data received will be analyzed to determine where the next gate is.

### code/scan_code_node.py
This node uses processed image data to scan the code shown. The node continuously applies red, green, and blue filters to the incoming images to determine when each color is being shown.

### dock/find_dock_codes_node.py
This node identifies the relative positions of each of the 3 docks based on the code of each (e.g. blue, red, green docks). The node applies red, green, and blue filters to the incoming images to determine the order of the docks

### dock/find_dock_entrances_node.py
This node finds the entrances of each dock using the voxels created by Autoware Auto's voxel grid node. The node first identifies the entrance directly in front by finding the two closest voxels which are not next to each other (e.g. on the same dock). It then estimates where the two other entrances could be by creating a line through the entrance in front. When the USV translates to the target dock, it validates the exact location of the entrance.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /local_costmap/voxel_grid | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | map | Used by euclidean clustering node to find buoys. Used by the voxel grid node to create voxels for identifying dock entrances. |
| usv/lidar_points | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | Used by the ground filter. |
| usv/camera_image | [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) | camera_link | Used by downscale_node. |
| /perception/get_code | [std_msgs/Int8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int8.html) | N/A | Requests that code scanning begins. |
| /perception/start_find_docks | [std_msgs/Int8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int8.html) | N/A | Requests that dock finding begins. |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /buoys/bounding_boxes | [autoware_auto_perception_msgs/BoundingBoxArray](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/BoundingBoxArray.idl) | map | Buoys located and classified by height. |
| /perception/code | [std_msgs/Int32MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32MultiArray.html) | N/A | Array of 3 integers representing the color sequence identified. 0 = red, 1 = green, 2 = blue. |
| /perception/dock_code_offsets | [std_msgs/Int32MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32MultiArray.html) | N/A | Array of 3 integers representing the offset of each dock's code from the center of the camera along the y-axis. |
| /perception/dock_ahead_entrance | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | base_link | 2 Points representing the 2 endpoints of the entrance ahead. |

## External Services
| Service | Service Type | Frame | Purpose |
|---------|--------------|-------|---------|
| channel | [virtuoso_msgs/Channel](/virtuoso_msgs/srv/Channel.srv) | front_left_camera_link | Finds the next channel. |

## Parameters

### ray_ground_classifier.yaml
Key parameter here is sensor_height_m, as this will defintely change between USVs. Unfortunately, no documentation for the parameters found online.

### euclidean_clustering.yaml
Unfortunately, no documentation for the parameters found online.

### voxel_grid_node.yaml
Unfortunately, no documentation for the parameters found online.

### lidar_processing.yaml

| Node | Parameter | Type | Description |
|------|----------------|------|-------------|
| processing_self_filter | radius | float | The minimum distance from the LIDAR a point must be for it to be treated as not part of the robot. Any points within this radius are filtered. |
| processing_shore_filter | x_min | float | The minimum x-coordinate a point must have in order to not be filtered. For example, an x_min of 0 would filter all points behind the LIDAR. |
| processing_shore_filter | y_min | float | The minimum y-coordinate a point must have in order to not be filtered. For example, a y_min of 0 would filter all points to the right of the LIDAR. |
| processing_shore_filter | y_max | float | The maximum y-coordinate a point must have in order to not be filtered. For example, a y_max of 0 would filter all points to the left of the LIDAR. |

## camera_processing.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| noise_filter | debug | bool | If true, debug messages will be published. |
| noise_filter | denoising_params | int[4] | [h, hColor, templateWindowSize, searchWindowSize] for cv2.fastNlMeansDenoisingColored. |
| resize | debug | bool | If true, debug messages will be published. |
| resize | resize_factor | int | Factor by which to resize (e.g. 2 will create an image 1/4 the original area). |

### buoys.yaml

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| perception_find_buoys | buoy_max_side_length | float | Any clusters with a length larger than this value will not be treated as a buoy. |
| perception_find_buoys | tall_buoy_min_z | float | Any buoys located with a top (of this buoy) above this value will be classified as a tall buoy. Note that this is the top relative to the lidar_link, not the base_link. |
| perception_find_buoys | buoy_loc_noise | float | Any clusters within this distance of each other will be treated as the same buoy. |

### code.yaml

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| perception_scan_code | debug | bool | If true, debug messages will be published. |
| perception_scan_code | red.lower1 | int[3] | First lower bound for the red hsv filter. |
| perception_scan_code | red.upper1 | int[3] | First upper bound for the red hsv filter. |
| perception_scan_code | red.lower2 | int[3] | Second lower bound for the red hsv filter. |
| perception_scan_code | red.upper2 | int[3] | Second upper bound for the red hsv filter. |
| perception_scan_code | green.lower | int[3] | Lower bound for the green hsv filter. |
| perception_scan_code | green.upper | int[3] | Upper bound for the green hsv filter. |
| perception_scan_code | blue.lower | int[3] | Lower bound for the blue hsv filter. |
| perception_scan_code | blue.upper | int[3] | Upper bound for the blue hsv filter. |
| perception_scan_code | code_loc_noise | float | Any rectangles with an origin within this distance of the previous rectaingles will be considered a match (for determining if a current color is being displayed).

### dock.yaml

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| perception_find_dock_codes | debug | bool | If true, debug messages will be published. |
| perception_find_dock_codes | red.lower1 | int[3] | First lower bound for the red hsv filter. |
| perception_find_dock_codes | red.upper1 | int[3] | First upper bound for the red hsv filter. |
| perception_find_dock_codes | red.lower2 | int[3] | Second lower bound for the red hsv filter. |
| perception_find_dock_codes | red.upper2 | int[3] | Second upper bound for the red hsv filter. |
| perception_find_dock_codes | green.lower | int[3] | Lower bound for the green hsv filter. |
| perception_find_dock_codes | green.upper | int[3] | Upper bound for the green hsv filter. |
| perception_find_dock_codes | blue.lower | int[3] | Lower bound for the blue hsv filter. |
| perception_find_dock_codes | blue.upper | int[3] | Upper bound for the blue hsv filter. |
| perception_find_dock_codes | code_axis_range | float | Maximum difference in dock code locations along x-axis. |
| perception_find_dock_entrances | debug | bool | If true, debug messages will be published. |
