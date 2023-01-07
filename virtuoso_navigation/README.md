# Virtuoso Navigation

## Contents
- [Nav2](#nav2)
- [Virtuoso Navigation Nodes](#virtuoso-navigation-nodes)
  - [waypoints_node.py](#waypoints\_nodepy)
  - [translate_node.py](#translate\_nodepy)
  - [station_keeping_node.py](#station\_keeping\_nodepy)
  - [rotate_node.py](#rotate\_nodepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [External Services](#external-services)
- [Parameters](#parameters)
  - [nav2.yaml](#nav2yaml)

## Nav2

Nav2 is an open-source package which is used by our navigation server for global path planning. Nav2 takes in odometry from the localization server and point clouds from the processing server, and it creates a global costmap. When we send it a goal in the map frame, it will generate a sequence of waypoints which the USV will follow. This path is passed to our controller server. Further documentation can be found on the [Nav2 Github](https://github.com/ros-planning/navigation2). 

## Virtuoso Navigation Nodes

### waypoints_node.py
This node takes in a sequence of waypoints in the map frame the USV needs to navigate to. Every time the USV arrives at a waypoint, it will generate a path to the next waypoint through Nav2. 

### translate_node.py
This node takes in a point in the base_link frame the USV needs to translate to. The point is transformed to the map frame and then passed into the waypoints_node as a single waypoint.

### station_keeping_node.py
This node upon request publishes a path with a single waypoint (the current pose) to the controller. The controller will then attempt to hold that pose.

### rotate_node.py
This node upon request rotates the USV by a requested number of radians.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /localization/odometry | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | odom | Used by Nav2 for generating costmap and waypoints_node to determine when to navigate to next waypoint. |
| /processing/lidar/points_shore_filtered | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | Used by Nav2 for generating costmap. |
| /navigation/set_path | [nav_msgs/Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) | map | Used by the waypoints_node. |
| /navigation/translate | [geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html) | base_link | Used by translate_node. |
| /navigation/station_keep | [std_msgs/Empty](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html) | N/A | Activates station keeping. Used by station_keeping_node. |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /navigation/plan | [nav_msgs/Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) | map | The global plan the USV should follow. Used by the controller server. |
| /navigation/success | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | map | Final pose the USV successfully navigates to from waypoint navigation. |
| /navigation/translate_success | [geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html) | base_link | Point USV successfully translates to. |

## External Services 

| Service | Service Type | Frame | Purpose |
|---------|--------------|-------|---------|
| rotate | [virtuoso_msgs/Rotate](/virtuoso_msgs/srv/Rotate.srv) | base_link | Rotates the USV a certain number of radians. |

## Parameters

### nav2.yaml
Parameters for tuning Nav2. A full configuration guide can be found [here](https://navigation.ros.org/configuration/index.html). 

