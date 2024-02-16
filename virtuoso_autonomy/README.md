# Virtuoso Autonomy

## Contents
- [Launching Tasks](#launching-tasks)
- [Virtuoso Nodes](#virtuoso-nodes)
  - [robotx/docking_node.py](#robotxdocking\_nodepy)
  - [robotx/enter_exit_node.py](#robotxenter\_exit\_nodepy)
  - [robotx/gymkhana_node.py](#robotxgymkhana\_nodepy)
  - [robotx/safety_check_node.py](#robotxsafety\_check\_nodepy)
  - [robotx/scan_code_node.py](#robotxscan\_code\_nodepy)
  - [roboboat/channel_nav_node.py](#roboboatchannel\_nav\_nodepy)
  - [roboboat/safety_check_node.py](#roboboatsafety\_check\_nodepy)
  - [roboboat/loop_node.py](#roboboatloop\_nodepy)
  - [roboboat/docking_node.py](#roboboatdocking\_nodepy)
  - [roboboat/ball_shooter_node.py](#roboboatball\_shooter\_nodepy)
  - [roboboat/water_shooter_node.py](#roboboatwater\_shooter\_nodepy)
  - [roboboat/semis_node.py](#roboboatsemis\_nodepy)
  - [roboboat/finals_node.py](#roboboatfinals\_nodepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [External Service Requests](#external-service-requests)
- [External Action Requests](#external-action-requests)
- [Parameters](#parameters)
  - [docking.yaml](#dockingyaml)
  - [gymkhana.yaml](#gymkhanayaml)
  - [channel_nav.yaml](#channel\_navyaml)
  - [safety_check.yaml](#safety\_checkyaml)
  - [loop.yaml](#loopyaml)
  - [water_shooter.yaml](#water\_shooteryaml)
  - [semis.yaml](#semisyaml)
  - [finals.yaml](#finalsyaml)

## Launching Tasks

Each task has 2 launch files. To launch a task, first launch the `[usv][task_name]_setup.launch.py` file in a terminal. Then, in a separate terminal, run the `[usv][task_name].launch.py` file. We launch the setup separatly from the autonomy node so that when testing on the water, we can simply launch only the autonomy node when we are ready to start autonomy.

## Virtuoso Nodes

Each node has a similar sturcture. There is a `[task_name]_states.py` file which contains the current state of the USV in the task. There is a `[task_name].py` file which handles the logic for executing a task given the current state and inputted data. Finally, there is a `[task_name]_node.py` file which handles the transfer of data from topics to `[task_name].py` and the transfer of data from `[task_name].py` to other topics. If there is only a `[task_name]_node.py` file, then the logic is also there.

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

### roboboat/channel_nav_node.py
Handles the Channel Navigation task. USV procedure:

1. Enable station keeping
2. Search for next gate
3. Navigate to next gate
4. Translate forward an arbitrary, customizable distance
5. Repeat steps 2-4 until navigated through all gates (customizable)

### roboboat/safety_check_node.py
Handles the first task for RoboBoat. USV procedure:

1. Enable station keeping
2. Search for next gate
3. Navigate to next gate
4. Translate forward an arbitrary, customizable distance
5. Repeat steps 2-4 until navigated through two gates

### roboboat/loop_node.py
Handles the Northwest Passage task of RoboBoat (task 4). USV procedure:

1. Enable station keeping
2. search for entrance gate
3. Navigate to entrance gate and record waypoint
4. Translate forward an arbitrary, customizable distance
5. Search for loop buoy
6. If no loop buoy found, navigate 10 meters forward and go to step 5
7. Find three waypoints around the buoy for the USV to follow
8. Add the three waypoints and the gate midpoint waypoint to the path
9. After completing the path, navigate an arbitrary, customizable distance (to go through the gate)

### roboboat/docking_node.py
Handles the docking task of RoboBoat (task 3). USV procedure:

1. Enable station keeping
2. Translate left or right until in front of correct dock
3. Translate forward until within an arbitrary distance of the dock

### roboboat/ball_shooter_node.py
Handles the ball shooting task of RoboBoat (task 6). USV procedure:

1. Wait for the waypoint player to navigate to a preset waypoint
2. Request for the ball shooter to shoot

### roboboat/water_shooter_node.py
Handles the water shooting task of RoboBoat (task 7). USV procedure:

1. Wait for the waypoint player to navigate to a preset waypoint
2. Request for the water shooter to shoot

### roboboat/semis_node.py
Handles the semifinal run of RoboBoat. USV procedure:

0. Collect waypoints for navigating the entire course, stored as part of distinct tasks (see `virtuoso_localiztion` documentation on the `multi_tasks_waypoint_saver_node.py`)
1. Start the next task and request for the waypoint player to navigate to all waypoints for the current task
2. When task navigation completes, check if task has some additional functionality (docking for x seconds, shooting balls, or shooting water), and perform additional functionality if present
3. Return to step 1 unless already completed all tasks

### roboboat/finals_node.py
Handles the finals run of RoboBoat. USV procedure:

0. Estimate waypoints for the start of each task using Mission Planner
1. Start the next task and request for waypoint player to navigate to all waypoints for current task
2. Run the task using perception
3. Upon task autonomous completion (or timeout), return to step 1

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /localization/odometry | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | odom | See `virtuoso_localization`. |
| /navigation/success | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html) | map | See `virtuoso_navigation`. |
| /navigation/translate_success | [geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html) | base_link | See `virtuoso_navigation`. |
| /perception/dock_ahead_entrance | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | base_link | See `virtuoso_perception`. |
| /perception/code | [std_msgs/Int32MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32MultiArray.html) | N/A | See `virtuoso_perception`. |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /navigation/station_keep | [std_msgs/Empty](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html) | N/A | Enables station keeping. |
| /navigation/set_waypoints | [nav_msgs/Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) | map | Path USV should navigate along. |
| /navigation/translate | [geometry_msgs/Point](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html) | base_link | Point USV should translate to. |
| /controller_mode | [std_msgs/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) | N/A | Sets the controller mode. Options are 'waypointing' or 'manual'. |
| /controller/manual/cmd_vel | [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) | N/A | The twist command for the manual controller. |
| /controller/manual/cmd_torque | [std_msgs/Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html) | N/A | The torque command for the manual controller. |

## External Service Requests

| Service | Service Type | Frame | Purpose |
|---------|--------------|-------|---------|
| channel | [virtuoso_msgs/Channel](/virtuoso_msgs/srv/Channel.srv) | map | Finds the next channel. |
| rotate | [virtuoso_msgs/Rotate](/virtuoso_msgs/srv/Rotate.srv) | base_link | Rotates the USV by a certain number of radians. |
| shoot_water | [virtuoso_msgs/ShootWater](/virtuoso_msgs/srv/ShootWater.srv) | N/A | See `virtuoso_auxiliary`.
| {cam}/find_dock_placard_offsets | [virtuoso_msgs/DockCodesCameraPos](/virtuoso_msgs/srv/DockCodesCameraPos.srv) | {cam}_link | See `virtuoso_perception`. |

## External Action Requests

| Action | Action Type | Frame | Purpose |
|---------|--------------|-------|---------|
| task_waypoint_nav | [virtuoso_msgs/TaskWaypointNav](/virtuoso_msgs/action/TaskWaypointNav.action) | N/A | See `virtuoso_navigation`. |
| approach_target | [virtuoso_msgs/ApproachTarget](/virtuoso_msgs/action/ApproachTarget.action) | N/A | See `virtuoso_navigation`. |
| shoot_balls | [virtuoso_msgs/ShootBalls](/virtuoso_msgs/action/ShootBalls.action) | N/A | See `virtuoso_auxiliary`. |

## Parameters

### docking.yaml

Parameters different for RobotX. RobotX docking has not been updated yet and will not work.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| autonomy_docking | target_color | string | The color of the dock the USV should dock at. |
| autonomy_docking | camera | string | The name of the camera to use for docking (prefix of certain service calls). |
| autonomy_docking | centering_movement_tranches | float[] | Tranches for translational movement depending on where the code is relative to the center of the camera being used. Movement for each tranch specified in `centering_movement_values`. |
| autonomy_docking | centering_movement_values | float[] | Number of meters to translate depending on which tranche the code is in. |
| autonomy_docking | search_movement_value | float | Number of meters to translate in search of the code when it is not seen by the camera. |
| autonomy_docking | docking_entry_value | float | Number of meters to translate forward when entering the dock. Note: for finals run we did not use a hard-coded translate forward value but instead used the `approach_target` action. |

### gymkhana.yaml

Currently only for RobotX.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| autonomy_gymkhana | num_channels | int | Number of channels to navigate through before stopping. |

### channel_nav.yaml

Currently only for RoboBoat.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| autonomy_channel_nav | linear_x_factor | float | Factor the x-component of velocity supplied to the controller multiplied by. |
| autonomy_channel_nav | linear_y_factor | float | Factor the y-component of velocity supplied to the controller multiplied by. |
| autonomy_channel_nav | torque_factor | float | Factor the torque supplied to the controller multiplied by. |

### safety_check.yaml

Currently only for RoboBoat.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| autonomy_safety_check | gate_buoy_max_dist | float | The number of meters a buoy for a next gate can be from the USV. |
| autonomy_safety_check | extra_forward_nav | float | The number of meters to navigate forward after reaching the midpoint of a gate. |
| autonomy_safety_check | use_lidar | bool | Whether to use the lidar for identifying buoys. |
| autonomy_safety_check | use_camera | bool | Whether to use the cameras for identifying buoys. |

### loop.yaml

Currently only for RoboBoat.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| autonomy_loop | gate_use_lidar | bool | Whether to use the lidar for finding the gate buoys. |
| autonomy_loop | gate_use_camera | bool | Whether to use the camera for finding the gate buoys. |
| autonomy_loop | gate_max_buoy_dist | float | The number of meters a buoy for the gate can be from the USV. |
| autonomy_loop | gate_extra_forward_nav | float | The number of meters to navigate forward after reaching the midpoint of the gate. |
| autonomy_loop | loop_use_lidar | bool | Whether to use the lidar for finding the loop buoy. |
| autonomy_loop | loop_use_camera | bool | Whether to use the cameras for finding the loop buoy. |
| autonomy_loop | loop_max_buoy_dist | float | The number of meters the loop buoy can be from the USV. |
| autonomy_loop | looping_radius | float | The number of meters to keep away from the loop buoy during the looping. |

### water_shooter.yaml

Currently only for RoboBoat.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| autonomy_water_shooter | shoot_seconds | float | The number of seconds to shoot water. |

### semis.yaml

Currently only for RoboBoat.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| autonomy_semis | task_nums | int[] | The tasks to attempt and the order in which to attempt them. (e.g. [1, 4, 2] will attempt tasks 1, 2, and 4 in the order specified in the array). |
| autonomy_semis | docking_num | int | The task number corresponding to docking. |
| autonomy_semis | ball_shooter_num | int | The task number corresponding to ball shooting. |
| autonomy_semis | water_shooter_num | int | The task number corresponding to water shooting. |
| autonomy_semis | docking_secs | int | The number of seconds to station keep at the dock. |
| autonomy_semis | water_secs | float | The number of seconds to shoot water. |

### finals.yaml

Currently only for RoboBoat.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| autonomy_finals | task_nums | int[] | The tasks to attempt and the order in which to attempt them. (e.g. [1, 4, 2] will attempt tasks 1, 2, and 4 in the order specified in the array). |
| autonomy_finals | docking_secs | int | The number of seconds to station keep at the dock. | 
| autonomy_finals | water_secs | int | The number of seconds to shoot water. |
| autonomy_finals | t1_auto | bool | Performs the task autonomously if true, else go to the waypoints for the next task. |
| autonomy_finals | t2_auto | bool | Performs the task autonomously if true, else go to the waypoints for the next task. |
| autonomy_finals | t3_auto | bool | Performs the task autonomously if true, else go to the waypoints for the next task. |
| autonomy_finals | t4_auto | bool | Performs the task autonomously if true, else go to the waypoints for the next task. |
| autonomy_finals | t5_auto | bool | Performs the task autonomously if true, else go to the waypoints for the next task. |
| autonomy_finals | t6_auto | bool | Performs the task autonomously if true, else go to the waypoints for the next task. |
| autonomy_finals | t7_auto | bool | Performs the task autonomously if true, else go to the waypoints for the next task. |
| autonomy_finals | t8_auto | bool | Performs the task autonomously if true, else go to the waypoints for the next task. |
| autonomy_finals | t1_extra_forward_nav | float | The number of meters to navigate forward after reaching the midpoint of the gate. |
| autonomy_finals | t1_final_extra_forward_nav | float | The number of meters to navigate forward after reaching the final gate midpoint. |
| autonomy_finals | t1_gate_buoy_max_dist | float | The number of meters a buoy for the gate can be from the USV. |
| autonomy_finals | t2_trans_x | float | Number of meters to translate in the x direction to attempt task 2. |
| autonomy_finals | t2_trans_y | float | Number of meters to translate in the y direction to attempt task 2. |
| autonomy_finals | t3_target_color | string | The color of the dock the USV should dock at. |
| autonomy_finals | t3_camera | string | The camera to use for finding the correct dock. |
| autonomy_finals | t3_skip_line_up | bool | If true, skip lining up the USV in front of the correct dock and instead begin entering the dock. |
| autonomy_finals | t3_centering_movement_tranches | float[] | Tranches for translational movement depending on where the code is relative to the center of the camera being used. Movement for each tranch specified in `t3_centering_movement_values`. |
| autonomy_finals | t3_centering_movement_values | float[] | Number of meters to translate depending on which tranche the code is in. |
| autonomy_finals | t3_search_movement_value | float[] | Number of meters to translate in search of the code when it is not seen by the camera. | 
| autonomy_finals | t3_approach_dist | float | Number of meters to be away from the dock during docking. |
| autonomy_finals | t3_approach_scan_width | float | Width of the lidar scan to use for determining how far the dock is from the USV. |
| autonomy_finals | t4_gate_max_buoy_dist | float | The number of meters a buoy for the gate can be from the USV. | 
| autonomy_finals | t4_loop_max_buoy_dist | float | The number of meters the loop buoy can be from the USV. |
| autonomy_finals | t4_gate_extra_forward_nav | float | The number of meters to navigate forward after reaching the gate midpoint. |
| autonomy_finals | t4_final_extra_forward_nav | float | The number of meters to navigate forward after reaching the gate midpoint on exit. |
| autonomy_finals | t4_looping_radius | float | The number of meters to keep away from the loop buoy during the looping. |
| autonomy_finals | t6_approach_dist | float | The number of meters to be away from the ball shooting target before reversing. |
| autonomy_finals | t6_approach_scan_width | float | The width of the lidar scan to use for determining how far the target is from the USV. | 
| autonomy_finals | t6_back_up_dist | float | The number of meters to back up before firing the balls. |
| autonomy_finals | timeouts.t1_find_next_gate | int | Number of seconds before finding the next gate on task 1 times out. |
| autonomy_finals | timeouts.t3_code_search | int | Number of seconds before the code search for docking times out. |
| autonomy_finals | timeouts.t3_approach | int | Number of seconds before docking approach times out. |
| autonomy_finals | timeouts.t4_gate | int | Number of seconds before finding the entrance gate for buoy loop times out. |
| autonomy_finals | timeouts.t4_loop | int | Number of seconds before finding the loop buoy times out. |
| autonomy_finals | timeouts.t6_approach | int | Number of seconds before approach for task 6 times out. |
| autonomy_finals | timeouts.t6_rotate | int | Number of seconds before rotating 180 degrees for shooting times out. |
| autonomy_finals | timeouts.t6_back_up | int | Number of seconds before backing up for ball shooting times out. |
| auotnomy_finals | timeout_responses.t1_find_next_gate_trans | float | Number of meters to navigate forward if t1_find_next_gate times out. |
| autonomy_finals | timeout_responses.t4_gate_trans | float | Number of meters to navigate forward if t4_gate times out. |