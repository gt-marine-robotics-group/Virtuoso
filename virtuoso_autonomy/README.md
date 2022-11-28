# Virtuoso Autonomy

## Contents
- [Launching Tasks](#launching-tasks)
- [Virtuoso Nodes](#virtuoso-nodes)
  - [robotx/docking_node.py](#robotxdocking\_nodepy)
  - [robotx/enter_exit_node.py](#robotxenter\_exit\_nodepy)
  - [robotx/gymkhana_node.py](#robotxgymkhana\_nodepy)
  - [robotx/safety_check_node.py](#robotxsafety\_check\_nodepy)
  - [robotx/scan_code_node.py](#robotxscan\_code\_nodepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [docking.yaml](#dockingyaml)
  - [gymkhana.yaml](#gymkhanayaml)

## Launching Tasks

Each task has 2 launch files. To launch a task, first launch the `[usv][task_name]_setup.launch.py` file in a terminal. Then, in a separate terminal, run the `[usv][task_name].launch.py` file. We launch the setup separatly from the autonomy node so that when testing on the water, we can simply launch only the autonomy node when we are ready to start autonomy.

## Virtuoso Nodes

Each node has a similar sturcture. There is a `[task_name]_states.py` file which contains the current state of the USV in the task. There is a `[task_name].py` file which handles the logic for executing a task given the current state and inputted data. Finally, there is a `[task_name]_node.py` file which handles the transfer of data from topics to `[task_name].py` and the transfer of data from `[task_name].py` to other topics.

### robotx/docking_node.py

Handles the docking task. USV procedure: 
1. Enable station keeping
2. Find relative location of docks
3. Approach dock within x meters (customizable)
4. Translate in the y-direction to the correct dock entrance
5. Enter the dock 

### robotx/enter_exit_node.py

Handles the Enter and Exit task. USV procedure:
1. Enable station keeping
2. Navigate to random entrance (no hydrophone)
3. Navigate around looping buoy and return to current location

### robotx/gymkhana_node.py

Handles the Gymkhana task. USV procedure:
1. Enable station keeping
2. Search for next gate
3. Navigate to next gate
4. Repeat steps 2 and 3 until navigated through all gates (customizable)

### robotx/safety_check_node.py

Handles the Safety Check (not technically a task). USV procedure: 
1. Enable station keeping
2. Search for next gate
3. Navigate to next gate
4. Repeat steps 2 and 3 once more

### robotx/scan_code_node.py

Handles the Scan the Code task. USV procedure:
1. Enable station keeping
2. Request code scan

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /localization/odometry | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | odom | Used by enter_exit_node, gymkhana_node, and safety_check_node. |
| /buoys/bounding_boxes | [autoware_auto_perception_msgs/BoundingBoxArray](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/BoundingBoxArray.idl) | map | Used by enter_exit_node, gymkhana_node, and safety_check_node. |
| /navigation/success | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | map | Used by enter_exit_node, gymkhana_node, and safety_check_node. |
| /navigation/translate_success | [geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html) | base_link | Used by docking_node. |
| /perception/dock_code_offsets | [std_msgs/Int32MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32MultiArray.html) | N/A | Used by docking_node. |
| /perception/dock_ahead_entrance | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | base_link | Used by docking_node. |
| /perception/code | [std_msgs/Int32MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32MultiArray.html) | N/A | Used by scan_code_node. |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /navigation/station_keep | [std_msgs/Empty](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html) | N/A | Enables station keeping. |
| /navigation/set_path | [nav_msgs/Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) | map | Path USV should navigate along. |
| /navigation/translate | [geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html) | base_link | Point USV should translate to. |

## Parameters

### docking.yaml

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| autonomy_docking | target_dock_color | string | Color of the target dock. Can be "red", "blue", or "green". |
| autonomy_docking | dock_approach_distance | float | Distance to be from dock before translating to target dock. |

### gymkhana.yaml

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| autonomy_gymkhana | num_channels | int | Number of channels to navigate through before stopping. |

