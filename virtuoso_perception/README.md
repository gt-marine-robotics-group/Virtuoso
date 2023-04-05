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
  - [dock/find_dock_posts_node.py](#dockfind\_dock\_posts\_nodepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [External Services](#external-services)
- [Parameters](#parameters)
  - [camera_config.yaml](#camera\_configyaml)
  - [ray_ground_classifier.yaml](#ray\_ground\_classifieryaml)
  - [lidar_processing.yaml](#lidar\_processingyaml)
  - [camera_processing.yaml](#camera\_processingyaml)
  - [euclidean_clustering.yaml](#euclidean\_clusteringyaml)
  - [voxel_grid_node.yaml](#voxel\_grid\_nodeyaml)
  - [buoys.yaml](#buoysyaml)
  - [stereo.yaml](#stereoyaml)
  - [code.yaml](#codeyaml)
  - [dock.yaml](#dockyaml)

## Autoware Auto Nodes

### Ray Ground Classifier
This node applies a ground filter to the incoming PointCloud. Points that are not the ground (in our case the water) are further processed.

### Euclidean Clustering
This node finds clusters from the voxel grid created by Nav2. These clusters can then be interpreted as objects like buoys or docks.

### Voxel Grid
This node takes in the voxel grid created by Nav2 and creates voxels of those voxels. These are used to help identify the entrances to docks as, if tuned correctly, there can be about 1 voxel per entry point. Currently being phased out for a more robust solution to docking.

## Virtuoso Perception Nodes

### lidar_processing/self_filter_node.py
This node filters any points within a certain distance of the LIDAR. These points in most cases will be the USV itself.

### lidar_processing/shore_filter_node.py
This node filters any points less a certain x-values and outside a certain range along the y-axis. This is not necessarily a "shore filter" as it was intending to be, but will filter anything outside of our desired field of view.

### camera_processing/noise_filter_node.py
This node removes noise from an image. Because our environment tends to be quite sparse (water with fairly distinct objects on them like buoys), we prefer "over-filtering" noise from an image than not removing enough. Losing small details from incoming image data is not a big deal. Note that this node's service is not typically called, but instead the class is used in a different node in order to avoid sending unnecessarily sending images over the network.

### camera_processing/resize_node.py
This node resizes the incoming image so as to make image computations further down the stack less computationally expensive. Note that this node's service is not typically called, but instead the class is used in a different node in order to avoid sending unnecessarily sending images over the network.

### buoys/find_buoys_node.py
This node uses the euclidean clusters to determine the location of buoys (Lidar). Buoys are assigned a value corresponding to their height. Buoys with a value >= 1 are considered "tall" buoys.

### buoys/buoy_filter_node.py
This node takes processed image data and filters out objects that are not one of the buoy types specified. It returns the contours of these buoys.

### buoys/channel_node.py
This node, when requested, will identify a channel made from two buoys using either lidar or stereo vision or both (as requested). If no buoys are found, the service will return two null points. If only one buoy is found, the service will return two identical points. If two buoys are found, the service will return two distinct points.

### code/scan_code_node.py
This node uses processed image data to scan the code shown. The node continuously applies red, green, and blue filters to the incoming images to determine when each color is being shown.

### dock/find_dock_codes_node.py
This node identifies the relative positions of each of the 3 docks based on the code of each (e.g. blue, red, green docks). The node applies red, green, and blue filters to identify clusters of each color. It then checks that each color cluster is surrounded by the placard (identified with a color range). Depending on the service called, the node will either return the pixel offsets of each placard from the center of the image, the contours of each code found, or the number of codes of a certain color found.

### dock/find_dock_entrances_node.py
This node finds the entrances of each dock using the voxels created by Autoware Auto's voxel grid node. The node first identifies the entrance directly in front by finding the two closest voxels which are not next to each other (e.g. on the same dock). It then estimates where the two other entrances could be by creating a line through the entrance in front. When the USV translates to the target dock, it validates the exact location of the entrance. Currently being phased out for a more robust solution.

### dock/find_dock_posts_node.py
This node finds the location of the dock posts using stereo vision (for Roboboat currently). The left-most dock has a green marker, there are two white markers around the center dock, and the right-most dock has a red marker. Currently being phased out for a more robust solution.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| usv/lidar_points | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | Used by the ground filter. |
| usv/image_raw | [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) | camera_link | Used by downscale_node. |
| /perception/get_code | [std_msgs/Int8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int8.html) | N/A | Requests that code scanning begins. |
| /perception/start_find_docks | [std_msgs/Int8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int8.html) | N/A | Requests that dock finding begins. |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /perception/code | [std_msgs/Int32MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32MultiArray.html) | N/A | Array of 3 integers representing the color sequence identified. 0 = red, 1 = green, 2 = blue. |
| /perception/dock_ahead_entrance | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | base_link | 2 Points representing the 2 endpoints of the entrance ahead. |

## External Services

| Service | Service Type | Frame | Purpose |
|---------|--------------|-------|---------|
| channel | [virtuoso_msgs/Channel](/virtuoso_msgs/srv/Channel.srv) | map | Finds the next channel. |
| {cam}/count_dock_codes | [virtuoso_msgs/CountDockCodes](/virtuoso_msgs/srv/CountDockCodes.srv) | N/A | Counts the number of codes for a specific color (red, green, or blue). |
| {cam}/find_dock_placard_offsets | [virtuoso_msgs/DockCodesCameraPos](/virtuoso_msgs/src/DockCodesCameraPos.srv) | {cam}_link | Finds the placard offsets of each dock by finding the codes on each placard. |
| {cam}/dock_code_contours | [virtuoso_msgs/ImageBuoyFilter](/virtuoso_msgs/srv/ImageBuoyFilter) | {cam}_link | Finds the contours of each dock code. Returns them in the same way as the buoy filter, so same service type is used. Currently being phased out.

## Parameters

### camera_config.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| For multiple nodes | all_camera_base_topics | string[] | Base topics for all cameras. Will be used to determine publishers and subscribers for all camera data (e.g. base_camera_topic/image_raw, base_camera_topic/camera_info, base_camera_topic/resized, etc.).
| For multiple nodes | all_camera_matrices | float\[]\[9] | K matrix for [sensor_msgs/CameraInfo](#http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html). |
| For multiple nodes | all_camera_transforms | float\[]\[3] | X, Y, and Z translation from base_link to each camera. |
| For multiple nodes | all_camera_frames | string[] | Frames of each of the cameras. |
| For multiple nodes | bow_camera_base_topics | string[] | Base topics for cameras on the bow of the USV. Used by stereo nodes. |
| For multiple nodes | bow_camera_frames | string[] | Frames of cameras on bow of the USV. Used by stereo nodes. |

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
| perception_buoy_filter| debug | bool | If true, debug messages will be published. |
| perception_buoy_filter | filter_bounds.red.lower1 | int[3] | First lower bound for the red hsv filter. |
| perception_buoy_filter | filter_bounds.red.upper1 | int[3] | First upper bound for the red hsv filter. |
| perception_buoy_filter | filter_bounds.red.lower2 | int[3] | Second lower bound for the red hsv filter. |
| perception_buoy_filter | filter_bounds.red.upper2 | int[3] | Second upper bound for the red hsv filter. |
| perception_buoy_filter | filter_bounds.green.lower | int[3] | Lower bound for the green hsv filter. |
| perception_buoy_filter | filter_bounds.green.upper | int[3] | Upper bound for the green hsv filter. |
| perception_buoy_filter | filter_bounds.black.lower | int[3] | Lower bounds for the black hsv filter. |
| perception_buoy_filter | filter_bounds.black.upper | int[3] | Upper bound for the black hsv filter. |
| perception_buoy_filter | filter_bounds.yellow.lower | int[3] | Lower bound for the yellow hsv filter. |
| perception_buoy_filter | filter_bounds.yellow.upper | int[3] | Upper bound for the yellow hsv filter. |
| perception_buoy_filter | label_bounds.red.lower1 | int[3] | First lower bound for the red hsv classifier. |
| perception_buoy_filter | label_bounds.red.upper1 | int[3] | First upper bound for the red hsv classifier. |
| perception_buoy_filter | label_bounds.red.lower2 | int[3] | Second lower bound for the red hsv classifier. |
| perception_buoy_filter | label_bounds.red.upper2 | int[3] | Second upper bound for the red hsv classifier. |
| perception_buoy_filter | label_bounds.green.lower | int[3] | Lower bound for the green hsv classifier. |
| perception_buoy_filter | label_bounds.green.upper | int[3] | Upper bound for the green hsv classifier. |
| perception_buoy_filter | label_bounds.black.lower | int[3] | Lower bounds for the black hsv classifier. |
| perception_buoy_filter | label_bounds.black.upper | int[3] | Upper bounds for the black hsv classifier. |
| perception_buoy_filter | label_bounds.yellow.lower | int[3] | Lower bounds for the yellow hsv classifier. |
| perception_buoy_filter | label_bounds.yellow.upper | int[3] | Upper bounds for the yellow hsv classifier. |

### stereo.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| perception_buoy_stereo | debug | bool | If true, debug messages will be published. |
| perception_buoy_stereo | multiprocessing | bool | If true, each pair of contours will have its location in 3d space found in separate processes rather than sequentially. If true while debug is true, certain debug topics will not be published to. |

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
