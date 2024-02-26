# Virtuoso Perception

## Contents
- [Virtuoso Perception Nodes](#virtuoso-perception-nodes)
  - [lidar_processing/ground_filter_node.cpp](#lidar\_processingground\_filter\_nodecpp)
  - [lidar_processing/euclidean_clustering_node.cpp](#lidar\_processingeuclidean\_clustering\_nodecpp)
  - [lidar_processing/self_filter_node.cpp](#lidar\_processingself\_filter\_nodecpp)
  - [lidar_processing/shore_filter_node.cpp](#lidar\_processingshore\_filter\_nodecpp)
  - [lidar_processing/voxels_node.cpp](#lidar\_processingvoxels\_nodecpp)
  - [camera_processing/resize_node.py](#camera\_processingresize\_nodepy)
  - [camera_processing/noise_filter_node.py](#camera\_processingnoise\_filter\_nodepy)
  - [buoys/buoy_lidar_node.py](#buoysbuoy\_lidar\_nodepy)
  - [buoys/buoy_cam_filter_node.py](#buoysbuoy\_cam\_filter\_nodepy)
  - [buoys/channel_node.py](#buoyschannel\_nodepy)
  - [code/scan_code_node.py](#codescan\_code\_nodepy)
  - [dock/find_dock_codes_node.py](#dockfind\_dock\_codes\_nodepy)
  - [dock/find_dock_entrances_node.py](#dockfind\_dock\_entrances\_nodepy)
  - [dock/find_dock_posts_node.py](#dockfind\_dock\_posts\_nodepy)
  - [stereo/buoy_stereo_node.py](#stereobuoy\_stereo\_nodepy)
  - [stereo/dock_stereo_node.py](#stereodock\_stereo\_nodepy)
  - [yolo/yolo_node.py](#yoloyolo\_nodepy)
- [Other Algorithms of Note](#other-algorithms-of-note)
  - [Density Filter](#clusteringdensity\_filterpy)
  - [Color Filter](#utilsColorRangepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [External Services](#external-services)
- [Parameters](#parameters)
	- [ground_filter.yaml](#ground\_filteryaml)
	- [euclidean_clustering.yaml](#euclidean\_clusteringyaml)
  - [camera_config.yaml](#camera\_configyaml)
  - [lidar_config.yaml](#lidar\_configyaml)
  - [lidar_processing.yaml](#lidar\_processingyaml)
  - [camera_processing.yaml](#camera\_processingyaml)
  - [euclidean_clustering.yaml](#euclidean\_clusteringyaml)
  - [buoys.yaml](#buoysyaml)
  - [stereo.yaml](#stereoyaml)
  - [code.yaml](#codeyaml)
  - [dock.yaml](#dockyaml)
  - [dock_codes.yaml](#dock\_codesyaml)
  - [dock_posts.yaml](#dock\_postsyaml)


## Virtuoso Perception Nodes

### lidar_processing/ground_filter_node.cpp
This node filters the lidar pointcloud which is the ground. On the water, this node does not need to filter anything since the lidar does not reflect off the water. However, it is needed when tested on ground or in the new VRX simulation (for ROS 2 Humble) where lidar does reflect off the water.

### lidar_processing/euclidean_clustering_node.cpp
This node runs a euclidean clustering algorithm on the filtered point cloud to identify obstacles.

### lidar_processing/self_filter_node.cpp
This node filters any points within a certain distance of the LIDAR. These points in most cases will be the USV itself.

### lidar_processing/shore_filter_node.cpp
This node filters any points less a certain x-values and outside a certain range along the y-axis. This is not necessarily a "shore filter" as it was intending to be, but will filter anything outside of our desired field of view.

### lidar_processing/voxels_node.cpp
This node takes in Lidar data from the shore filter node and publishes a voxel grid (same PointCloud2 message type). The purpose of the voxel grid is to reduce the number of Lidar points processed by future perception algorithms while preserving the major features of the original point cloud.

### camera_processing/noise_filter_node.py
This node removes noise from an image. Because our environment tends to be quite sparse (water with fairly distinct objects on them like buoys), we prefer "over-filtering" noise from an image than not removing enough. Losing small details from incoming image data is not a big deal. Note that this node's service is not typically called, but instead the class is used in a different node in order to avoid sending unnecessarily sending images over the network.

### camera_processing/resize_node.py
This node resizes the incoming image so as to make image computations further down the stack less computationally expensive. Note that this node's service is not typically called, but instead the class is used in a different node in order to avoid sending unnecessarily sending images over the network.

### buoys/buoy_lidar_node.py
This node uses the euclidean clusters to determine the location of buoys (Lidar). Buoys are assigned a value corresponding to their height. Buoys with a value >= 1 are considered "tall" buoys.

### buoys/buoy_cam_filter_node.py
This node takes processed image data and filters out objects that are not one of the buoy types specified. First a color filter is applied. Then, two clustering algorithms can be used to identify the buoys. The first is CV2's find_contours function. The second is a DBSCAN-like algorithm which we have found more success with (check the Other Algorithms of Note section). It returns the contours of these buoys.

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

### stereo/buoy_stereo_node.py
This node first identifies the buoys in each camera by calling the service found in `buoy_cam_filter_node` for each camera. Then, it pairs clusters on the left camera with clusters on the right camera. Using the midpoint of each cluster, it then uses trigonometry to find the x and y location of the object relative to the left camera. We are able to do this by knowing the location of both cameras relative to each other and the focal lengths of the cameras. While results have been stellar in simulation and in the lab, results have been mixed on the water. Thus, we will probably be phasing our basic stereo vision implementation and instead purchase an outdoor stereo camera.

### stereo/dock_stereo_node.py
This node first identifies the codes of the dock by calling the service in the `find_dock_codes_node` which returns the contours of each code (this is not an external service listed below). Then, similar to buoy stereo, it matches midpoints and does trigonometry to determine to position of the two farthest codes. We can then use these two poses to orient the USV with the dock. Results have been mixed even in simulation, so we will be phasing this out in favor of an outdoor stereo camera.

### yolo/yolo_node.py
This node uses a YOLOv8 model to detect objects around the USV from a camera. The model's detected objects can be useful for multiple different tasks (e.g. buoys, docks, ball shooting target).

## Other Algorithms of Note

### clustering/density_filter.py
Inspired by the DBSCAN clustering algorithm typically used for machine learning applications, this algorithm finds clusters of pixels by a similar process. Any grayscaled-pixels which have a value greater than 10 are treated as points on a 2d-plane. We treat the distance between the midpoints of two adjacent pixels in the image as 1. From this "graph" (really just a black and white image where the white pixels are our points), we create clusters using two parameters: epsilon and min_pts. For a point to be added to a cluster permanently, it must have at least min_pts points surrounding it no further than epsilon units away. Once these clusters have been created, clusters that are too large or too small for us to consider the object desired can be filtered out by length and width.

### utils/ColorFilter.py
Colors for objects of choice, such as specifically colored buoys or dock codes can be assigned a range of colors. These color ranges are specified as HSV colors rather than RGB or some other color formatting. We HSV colors because it is easier to tune our color filter manually. Ideally, we would find a way to automate the tuning of our color filter. Yes, this file should not be camel cased.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| {prefix}/points | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | Used by the ground filter. |
| {prefix}/image_raw | [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) | Frames specified in `camera_config.yaml` in `virtuoso_perception`. | Used by downscale_node. |
| /perception/get_code | [std_msgs/Int8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int8.html) | N/A | Requests that code scanning begins. |
| /perception/start_find_docks | [std_msgs/Int8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int8.html) | N/A | Requests that dock finding begins. |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /perception/lidar/voxels | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | Voxel grid of the processed lidar data. Used in mapping for obstacle detection.
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

### ground_filter.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| perception_ground_filter | frame_for_filtering | string | The frame to compute the ground filter from. Normally, this can just be left empty and the filtering will be done from the incoming lidar frame. However, in the new VRX simulation, because the lidar reflects off the water, we must do the filtering relative to the odom frame so that we account for the roll and pitch of the USV. |
| perception_ground_filter | frame_height | float | The height of the frame above the water. Normally, this is the distance from the base of the lidar to the water. |

### euclidean_clustering.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| perception_euclidean_clustering | cluster_tolerance | float | The maximum distance a point in a cluster can be from its nearest point in the cluster. |
| perception_euclidean_clustering | min_cluster_size | int | The minimum number of points in a cluster. |
| perception_euclidean_clustering | max_cluster_size | int | The maximum number of points in a cluster. |

### camera_config.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| For multiple nodes | all_camera_base_topics | string[] | Base topics for all cameras. Will be used to determine publishers and subscribers for all camera data (e.g. base_camera_topic/image_raw, base_camera_topic/camera_info, base_camera_topic/resized, etc.).
| For multiple nodes | all_camera_matrices | float\[]\[9] | K matrix for [sensor_msgs/CameraInfo](#http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html). |
| For multiple nodes | all_camera_transforms | float\[]\[3] | X, Y, and Z translation from base_link to each camera. |
| For multiple nodes | all_camera_frames | string[] | Frames of each of the cameras. |
| For multiple nodes | bow_camera_base_topics | string[] | Base topics for cameras on the bow of the USV. Used by stereo nodes. |
| For multiple nodes | bow_camera_frames | string[] | Frames of cameras on bow of the USV. Used by stereo nodes. |

### lidar_config.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| For multiple nodes | all_lidar_types | string[] | Types of lidar used; indices should match with other parameters. |
| For multiple nodes | all_lidar_base_topics | string[] | Base topics for all lidars. Will be used to determine publishers and subscribers for Lidars. |
| For multiple nodes | all_lidar_transforms | float\[]\[3] | X, Y, and Z translation from base_link to each Lidar. |
| For multiple nodes | all_lidar_frames | string[] | Frames of each Lidar. |

### lidar_processing.yaml

| Node | Parameter | Type | Description |
|------|----------------|------|-------------|
| processing_self_filter | radius | float | The minimum distance from the LIDAR a point must be for it to be treated as not part of the robot. Any points within this radius are filtered. |
| processing_shore_filter | x_min | float | The minimum x-coordinate a point must have in order to not be filtered. For example, an x_min of 0 would filter all points behind the LIDAR. |
| processing_shore_filter | y_min | float | The minimum y-coordinate a point must have in order to not be filtered. For example, a y_min of 0 would filter all points to the right of the LIDAR. |
| processing_shore_filter | y_max | float | The maximum y-coordinate a point must have in order to not be filtered. For example, a y_max of 0 would filter all points to the left of the LIDAR. |
| processing_voxels | debug | bool | Whether to print debug messages to the terminal. |
| processing_voxels | leaf_size.x | float | Set the voxel grid leaf size. |
| processing_voxels | leaf_size.y | float | Set the voxel grid leaf size. |
| processing_voxels | leaf_size.z | float | Set the voxel grid leaf size. |

## camera_processing.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| For multiple nodes | debug | bool | If true, debug messages will be published. |
| For multiple nodes | denoising_params | int[4] | [h, hColor, templateWindowSize, searchWindowSize] for cv2.fastNlMeansDenoisingColored. |
| For multiple nodes | resize_factor | int | Factor by which to resize (e.g. 2 will create an image 1/4 the original area). |

### buoys.yaml

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| perception_buoy_lidar | always_run | bool | If true, the node will publish the buoys it finds every time it gets new bounding boxes from Autoware Auto. If false, a service must be called for it to search for buoys. |
| perception_buoy_lidar | buoy_max_side_length | float | Any clusters with a length larger than this value will not be treated as a buoy. |
| perception_buoy_lidar | tall_buoy_min_z | float | Any buoys located with a top (of this buoy) above this value will be classified as a tall buoy. Note that this is the top relative to the lidar_link, not the base_link. |
| perception_buoy_lidar | buoy_loc_noise | float | Any clusters within this distance of each other will be treated as the same buoy. |
| perception_buoy_cam_filter| debug | bool | If true, debug messages will be published. |
| perception_buoy_cam_filter | filter_bounds.red.lower1 | int[3] | First lower bound for the red hsv filter. |
| perception_buoy_cam_filter | filter_bounds.red.upper1 | int[3] | First upper bound for the red hsv filter. |
| perception_buoy_cam_filter | filter_bounds.red.lower2 | int[3] | Second lower bound for the red hsv filter. |
| perception_buoy_cam_filter | filter_bounds.red.upper2 | int[3] | Second upper bound for the red hsv filter. |
| perception_buoy_cam_filter | filter_bounds.green.lower | int[3] | Lower bound for the green hsv filter. |
| perception_buoy_cam_filter | filter_bounds.green.upper | int[3] | Upper bound for the green hsv filter. |
| perception_buoy_cam_filter | filter_bounds.black.lower | int[3] | Lower bounds for the black hsv filter. |
| perception_buoy_cam_filter | filter_bounds.black.upper | int[3] | Upper bound for the black hsv filter. |
| perception_buoy_cam_filter | filter_bounds.yellow.lower | int[3] | Lower bound for the yellow hsv filter. |
| perception_buoy_cam_filter | filter_bounds.yellow.upper | int[3] | Upper bound for the yellow hsv filter. |
| perception_buoy_cam_filter | label_bounds.red.lower1 | int[3] | First lower bound for the red hsv classifier. |
| perception_buoy_cam_filter | label_bounds.red.upper1 | int[3] | First upper bound for the red hsv classifier. |
| perception_buoy_cam_filter | label_bounds.red.lower2 | int[3] | Second lower bound for the red hsv classifier. |
| perception_buoy_cam_filter | label_bounds.red.upper2 | int[3] | Second upper bound for the red hsv classifier. |
| perception_buoy_cam_filter | label_bounds.green.lower | int[3] | Lower bound for the green hsv classifier. |
| perception_buoy_cam_filter | label_bounds.green.upper | int[3] | Upper bound for the green hsv classifier. |
| perception_buoy_cam_filter | label_bounds.black.lower | int[3] | Lower bounds for the black hsv classifier. |
| perception_buoy_cam_filter | label_bounds.black.upper | int[3] | Upper bounds for the black hsv classifier. |
| perception_buoy_cam_filter | label_bounds.yellow.lower | int[3] | Lower bounds for the yellow hsv classifier. |
| perception_buoy_cam_filter | label_bounds.yellow.upper | int[3] | Upper bounds for the yellow hsv classifier. |
| perception_buoy_cam_filter | clustering_method | string | Either cluster buoys using "DENSITY" or "CV2_CONTOURS". |
| perception_buoy_cam_filter | max_cluster_height | int | Max cluster height in pixels (for Density filter). |
| perception_buoy_cam_filter | min_cluster_height | int | Min cluster height in pixels (for Density filter). |
| perception_buoy_cam_filter | max_cluster_width | int | Max cluster width in pixels (for Density filter). |
| perception_buoy_cam_filter | min_cluster_width | int | Min cluster width in pixels (for Density filter). |
| perception_buoy_cam_filter | epsilon | float | DBSCAN clustering epsilon. |
| perception_buoy_cam_filter | min_pts | int | DBSCAN clustering min_pts. |
| perception_buoy_cam_filter | buoy_border_px | int | When determining the color of a buoy, ignore any pixels "x" pixels from a border (for Cv2 contours). |
| perception_buoy_cam_filter | buoy_px_color_sample_size | int | Proportion of pixels within a cluster to sample to determine the buoy's color (for both clustering methods). |
| perception_buoy_cam_filter | use_resize | bool | If true, resize the image before clustering. |
| perception_buoy_cam_filter | use_noise_filter | bool | If true, apply a noise filter before clustering. |

### stereo.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| perception_buoy_stereo | debug | bool | If true, debug messages will be published. |
| perception_buoy_stereo | multiprocessing | bool | If true, each pair of contours will have its location in 3d space found in separate processes rather than sequentially. If true while debug is true, certain debug topics will not be published to. |
| perception_dock_stereo | debug | bool | If true, debug messages will be published. |
| perception_dock_stereo | multiprocessing | bool | If true, each pair of contours will have its location in 3d space found in separate processes rather than sequentially. If true while debug is true, certain debug topics will not be published to. |

### code.yaml

Only used for RobotX.

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

Only used for RobotX.

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

### dock_codes.yaml

Only used for RoboBoat.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| For multiple nodes (left and right camera). | debug | bool | If true, debug messages will be published. |
| For multiple nodes (left and right camera). | max_cluster_height | int | Max cluster height in pixels (for Density filter). |
| For multiple nodes (left and right camera). | min_cluster_height | int | Min cluster height in pixels (for Density filter). |
| For multiple nodes (left and right camera). | max_cluster_width | int | Max cluster width in pixels (for Density filter). |
| For multiple nodes (left and right camera). | min_cluster_width | int | Min cluster width in pixels (for Density filter). |
| For multiple nodes (left and right camera). | epsilon | float | DBSCAN clustering epsilon. |
| For multiple nodes (left and right camera). | min_pts | int | DBSCAN clustering min_pts. |
| For multiple nodes (left and right camera). | code_bounds.red.lower1 | int[3] | First lower bound for the red hsv filter. |
| For multiple nodes (left and right camera). | code_bounds.red.upper1 | int[3] | First upper bound for the red hsv filter. |
| For multiple nodes (left and right camera). | code_bounds.red.lower2 | int[3] | Second lower bound for the red hsv filter. |
| For multiple nodes (left and right camera). | code_bounds.red.upper2 | int[3] | Second upper bound for the red hsv filter. |
| For multiple nodes (left and right camera). | code_bounds.green.lower | int[3] | Lower bound for the green hsv filter. |
| For multiple nodes (left and right camera). | code_bounds.green.upper | int[3] | Upper bound for the green hsv filter. |
| For multiple nodes (left and right camera). | code_bounds.blue.lower | int[3] | Lower bounds for the blue hsv filter. |
| For multiple nodes (left and right camera). | code_bounds.blue.upper | int[3] | Upper bound for the blue hsv filter. |
| For multiple nodes (left and right camera). | code_px_color_sample_size | int | Proportion of pixels within a cluster to sample to determine the code's color. |
| For multiple nodes (left and right camera). | placard_bounds.lower | int[3] | Lower bounds for the placard hsv filter. |
| For multiple nodes (left and right camera). | placard_bounds.upper | int[3] | Upper bound for the placard hsv filter. |
| For multiple nodes (left and right camera). | placard_color_search_range | int | Max distance away from border of each contour to search for the placard background. |
| For multiple nodes (left and right camera). | min_placard_prop_between_codes | float | Min proportion of placard background surrounding a cluster. |
| For multiple nodes (left and right camera). | use_resize | bool | If true, resize the image before clustering. |
| For multiple nodes (left and right camera). | use_noise_filter | bool | If true, apply a noise filter before clustering. |

### dock_posts.yaml

Only used for RoboBoat.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| For multiple nodes (left and right camera). | debug | bool | If true, debug messages will be published. |
| For multiple nodes (left and right camera). | max_cluster_height | int | Max cluster height in pixels (for Density filter). |
| For multiple nodes (left and right camera). | min_cluster_height | int | Min cluster height in pixels (for Density filter). |
| For multiple nodes (left and right camera). | max_cluster_width | int | Max cluster width in pixels (for Density filter). |
| For multiple nodes (left and right camera). | min_cluster_width | int | Min cluster width in pixels (for Density filter). |
| For multiple nodes (left and right camera). | epsilon | float | DBSCAN clustering epsilon. |
| For multiple nodes (left and right camera). | min_pts | int | DBSCAN clustering min_pts. |
| For multiple nodes (left and right camera). | post_bounds.red.lower1 | int[3] | First lower bound for the red hsv filter. |
| For multiple nodes (left and right camera). | post_bounds.red.upper1 | int[3] | First upper bound for the red hsv filter. |
| For multiple nodes (left and right camera). | post_bounds.red.lower2 | int[3] | Second lower bound for the red hsv filter. |
| For multiple nodes (left and right camera). | post_bounds.red.upper2 | int[3] | Second upper bound for the red hsv filter. |
| For multiple nodes (left and right camera). | post_bounds.green.lower | int[3] | Lower bound for the green hsv filter. |
| For multiple nodes (left and right camera). | post_bounds.green.upper | int[3] | Upper bound for the green hsv filter. |
| For multiple nodes (left and right camera). | post_bounds.blue.lower | int[3] | Lower bounds for the blue hsv filter. |
| For multiple nodes (left and right camera). | post_bounds.blue.upper | int[3] | Upper bound for the blue hsv filter. |
| For multiple nodes (left and right camera). | post_px_color_sample_size | int | Proportion of pixels within a cluster to sample to determine the post's color. |
| For multiple nodes (left and right camera). | post_px_min_density | float | Minimum density of pixels in the cluster (i.e. number of non-filtered pixels / number of pixels). |
| For multiple nodes (left and right camera). | placard_bounds.lower | int[3] | Lower bounds for the placard hsv filter. |
| For multiple nodes (left and right camera). | placard_bounds.upper | int[3] | Upper bound for the placard hsv filter. |
| For multiple nodes (left and right camera). | placard_color_search_range | int | Max distance away from border of each contour to search for the placard background. |
| For multiple nodes (left and right camera). | max_placard_prop | float | Max proportion of placard background surrounding a cluster. |
| For multiple nodes (left and right camera). | use_resize | bool | If true, resize the image before clustering. |
| For multiple nodes (left and right camera). | use_noise_filter | bool | If true, apply a noise filter before clustering. |