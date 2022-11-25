
## Nav2

Nav2 is an open-source package which is used by our navigation server for global path planning. Nav2 takes in odometry from the localization server and point clouds from the processing server, and it creates a global costmap. When we send it a goal in the map frame, it will generate a sequence of waypoints which the USV will follow. This path is passed to our controller server. Further documentation can be found on the [Nav2 Github](https://github.com/ros-planning/navigation2). 

## Virtuoso Navigation Nodes

### waypoints_node.py
This node takes in a sequence of waypoints in the map frame the USV needs to navigate to. Every time the USV arrives at a waypoint, it will generate a path to the next waypoint through Nav2. 

### translate_node.py
This node takes in a point in the base_link frame the USV needs to translate to. The point is transformed to the map frame and then passed into the waypoints_node as a single waypoint.

### station_keeping_node.py
This node upon request publishes a path with a single waypoint (the current pose) to the controller. The controller will then attempt to hold that pose.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /navigation/set_path | nav_msgs/Path | map | Waypoints to be navigated to. Used by the waypoints_node. |
