# Virtuoso Navigation

## Contents
- [Virtuoso Navigation Nodes](#virtuoso-navigation-nodes)
  - [waypoints_node.py](#waypoints\_nodepy)
  - [translate_node.py](#translate\_nodepy)
  - [station_keeping_node.py](#station\_keeping\_nodepy)
  - [rotate_node.py](#rotate\_nodepy)
  - [approach_target_node.py](#approach\_target\_nodepy)
  - [waypoint_player_node.py](#waypoint\_player\_nodepy)
  - [multi_tasks_waypoint_player_node.py](#multi\_tasks\_waypoint\_player\_nodepy)
- [Planners](#planners)
  - [Straight Path](#straightpathpy)
  - [RRT](#rrtpy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [External Services](#external-services)
- [External Actions](#external-actions)
- [Parameters](#parameters)
  - [waypoints.yaml](#waypointsyaml)
  - [rotate.yaml](#rotateyaml)

## Virtuoso Navigation Nodes

### waypoints_node.py
This node takes in a sequence of waypoints in the map frame the USV needs to navigate to. Every time the USV arrives at a waypoint, it will generate a path to the next waypoint through one of our planners. When finised with all waypoints, the node will publish a success message.

### translate_node.py
This node takes in a point in the base_link frame the USV needs to translate to. The point is transformed to the map frame and then passed into the `waypoints_node` as a single waypoint. When translation has finished, the node will publish a success message.

### station_keeping_node.py
This node upon request publishes a path with a single waypoint (the current pose) to the controller. The controller will then attempt to hold that pose.

### rotate_node.py
This node upon request rotates the USV by a requested number of radians.

### approach_target_node.py
Using the lidar data from perception, on a request, the node will navigate the USV to the requested distance from the object in front of it. Additionally, width of the lidar scan can be specified in the request along with a timeout.

### waypoint_player_node.py
Check the `virtuoso_localization` documentation on the `waypoint_saver_node.py` to see how waypoints are saved. The player will look inside the `~/mrg/waypoints_raw` directory for the `points_{file_num}.yaml` file containing the the waypoints to follow, where `file_num` is a parameter for the node. If `file_num` is not provided, it will simply find the most recent waypoints file (i.e. the one with the highest number). The pseudo-code below explains how the player navigates through each waypoint:
- while not finished with all gps waypoints:
  - convert current gps waypoint to the map frame, no need to change orientation from the file
  - send a path to the `waypoints_node` containing the map frame waypoint
  - when `waypoints_node` sends a success message, repeat for the next gps waypoint

When finished with all waypoints, the player will publish a success message.

### multi_tasks_waypoint_player_node.py
Check the `virtuoso_localization` documentation on the `multi_tasks_waypoint_saver_node.py` to see how multi-task waypoints are saved. The node takes a request specifying which task's waypoints the USV should navigate through. The waypoints are saved in a `points_{file_num}.yaml` file inside of `~/mrg/semis_waypoints`. If no file_num is specified as a paramter when the node is launched, the node will use the most recent waypoints file (i.e. the one with the highest number). The node will then find the waypoints for the requested task in the file and navigate through them, similar to how the `waypoint_player_node` does. Upon success, the action returns.

## Planners

### StraightPath.py
This is the planner of last resort. It will simply plan a straight line from the current robot pose to the goal pose, with waypoints spaced 0.2 meters apart. Obstacles will be ignored. 

### RRT.py
This planner is an implementation of the rapidly exploring random trees algorithm. The tree starting from the current pose grows randomly and avoids any obstacles in the costmap. Once a path to the goal is found, the path is smoothed by removing waypoints which would lead to a new path that does not intersect obstacles. See the `waypoints.yaml` parameter descriptions to see how it can be tuned.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /localization/odometry | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | odom | Used by Nav2 for generating costmap and waypoints_node to determine when to navigate to next waypoint. |
| /perception/lidar/points_shore_filtered | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | Used by Nav2 for generating costmap. |
| /navigation/set_waypoints | [nav_msgs/Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) | map | Used by the waypoints_node. |
| /navigation/translate | [geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html) | base_link | Used by translate_node. |
| /navigation/station_keep | [std_msgs/Empty](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html) | N/A | Activates station keeping. Used by station_keeping_node. |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /navigation/plan | [nav_msgs/Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) | map | The global plan the USV should follow. Used by the controller server. |
| /navigation/success | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | map | Final pose the USV successfully navigates to from waypoint navigation. |
| /navigation/translate_success | [geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html) | base_link | Point USV successfully translates to. |
| /navigation/waypoint_player/success | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | map | Final pose the USV navigates to using the waypoint player. |

## External Services 

| Service | Service Type | Frame | Purpose |
|---------|--------------|-------|---------|
| rotate | [virtuoso_msgs/Rotate](/virtuoso_msgs/srv/Rotate.srv) | base_link | Rotates the USV a certain number of radians. |

## External Actions

| Action | Action Type | Frame | Purpose |
|---------|--------------|-------|---------|
| approach_target | [virtuoso_msgs/ApproachTarget](/virtuoso_msgs/action/ApproachTarget.action) | N/A | USV approaches the object in front of it to a requested number of meters. |
| task_waypoint_nav | [virtuoso_msgs/TaskWaypointNav](/virtuoso_msgs/action/TaskWaypointNav.action) | N/A | USV navigates to the waypoints for the requested task. |

## Parameters

### waypoints.yaml

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| navigation_waypoints | debug | bool | If true, debug messages may be printed to the terminal or sent to debug topics. |
| navigation_waypoints | only_translate | bool | If true, they waypoint node will send a command to translate to the controller whenever it sends the controller a path. |
| navigation_waypoints | goal_dist_tolerance | float | The distance from the goal necessary for the waypoint node to decide that it has reached the waypoint. |
| navigation_waypoints | goal_rotation_tolerance | float | The maximum yaw difference in radians that the USV can be from the target orientation for the USV to be at the waypoint. |
| navigation_waypoints | planner | string | Planner to use. Valid options currently are "STRAIGHT" and "RRT". |
| navigation_waypoints | inflation_layer | float | Size of the inflation layer to pad around obstacles. Ensures paths are not too close to obstacles. |
| navigation_waypoints | rrt.step_dist | float | Size of the maximum step distance in the tree. The step distance is the distance from the new node in the tree to its neighbor. |
| navigation_waypoints | rrt.line_collision_check_granularity | float | At what granularity to check for collisions along the path to any obstacles. For example, if set to 1, each meter along the path the planner will check for a collision with obstacles to determine if the path is valid. | 
| navigation_waypoints | rrt.debug_iteration_time | float | When debug is set to true, the RRT planner will publish a tree that can be viewed in RVIZ. To see the tree grow at a slower speed, this parameter should be increased. If set to zero, the planner will run at normal speed. |

### rotate.yaml

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| navigation_rotate | goal_tolerance | float | The goal tolerance in radians for the node to send a response. |
