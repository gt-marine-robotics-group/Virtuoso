# Virtuoso Controller

## Contents
- [Virtuoso Nodes](#virtuoso-nodes)
  - [choose_pid_node.py](#choose\_pid\_nodepy)
  - [cmd_vel_generator_node.py](#cmd\_vel\_generator\_nodepy)
  - [velocity_pid_node.py](#velocity\_pid\_nodepy)
  - [basic_pid_node.py](#basic\_pid\_nodepy)
  - [motor_cmd_generator_node.py](#motor\_cmd\_generator\_nodepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [pidgains.yaml](#pidgainsyaml)

## Virtuoso Nodes

### choose_pid_node.py
This node sends the appropriate waypoints to the Basic PID, and it also decides whether to use the velocity PID for translational movement or the Basic PID.

### cmd_vel_generator_node.py

The outer loop takes the vehicle position and target path as its inputs. It calculates a desired velocity as a sum of two vectors. The first of these is a vector directly towards the line between the closest point on the path and the next point on the path. The second is the vector from the closest point on the path to the next point on the path. These vectors are scaled such that if the vehicle is further from the path, the vector towards the path is favored. In addition, the velocity magnitude is decreased if the vehicle is pointing away from the target velocity so that the vehicle can rotate itself to face in the direction of travel. This is preferred since the hydrodynamics of the vehicle favor motion in the surge direction. 

### velocity_pid_node.py

The main inner loop PID controller takes in the vehicle’s current velocity, target velocity, and target attitude and outputs a goal X and Y force as well as torque. 

The target attitude is the direction such that the vehicle’s +x axis is parallel to the desired velocity until the vehicle comes within a specified distance of its target, at which point the goal attitude specified by navigation package is set as the target. 

### basic_pid_node.py

The last-meter PID controller takes in vehicle velocity, position, and attitude as well as target position and attitude and outputs X and Y force as well as torque commands. The last-meter PID controller is implemented so that station keeping can be tuned separately from the vehicle behavior between goals. 

### motor_cmd_generator_node.py

The control mixer is what sends individual thruster commands based on target X and Y force as well as torque. The rear thruster commands are scaled down in magnitude to not induce excessive torque with Y force commands. In addition, the commands to all thrusters are scaled so that the maximum magnitude is 1.0, but the ratios of all thruster commands are kept. Can choose between the X and H drive configurations. In the X drive configuration, the side motors will output to 0. 

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /localization/odometry | [sensor_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | odom | Used by choose_pid_node, basic_pid_node, cmd_vel_generator_node, and velocity_pid_node. |
| /navigation/plan | [nav_msgs/Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) | map | Used by choose_pid_node. |
| /controller/is_translation | [std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html) | N/A | If true, vehicle holds its final orientation while moving instead of pointing in the direction of travel. Affects choose_pid and cmd_vel_generator|


## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| usv motors | [std_msgs/Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html) | N/A | Thrust for each motor on the USV. |

## Parameters

### pidgains.yaml

Note that the PID gain parameters are currently multiplied by the baked-in PID gains in the PID nodes to get the actual PID gains used. So a 0.5 in the config file would result in half the built-in gain.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| controller_basic_PID | basic_kp_x | float | Proportional factor of basic PID in the vehicle x direction. |
| controller_basic_PID | basic_kd_x | float | Derivate factor of basic PID in the vehicle x direction. |
| controller_basic_PID | basic_ki_x | float | Integral factor of basic PID in the vehicle x direction. |
| controller_basic_PID | basic_kp_y | float | Proportional factor of basic PID in the vehicle y direction. |
| controller_basic_PID | basic_kd_y | float | Derivate factor of basic PID in the vehicle y direction. |
| controller_basic_PID | basic_ki_y | float | Integral factor of PID in the vehicle y direction. |
| controller_basic_PID | basic_rotate_kp | float | Proportional gain factor of basic PID in rotation. |
| controller_basic_PID | basic_rotate_kd | float | Derivative gain factor of basic PID in rotation. |
| controller_basic_PID | basic_rotate_ki | float | Integral gain factor of basic PID in rotation. |
| controller_velocity_PID | velocity_k_drag_x | float | Feed-forward drag gain factor of velocity PID in the vehicle x direction. |
| controller_velocity_PID | velocity_k_error_x | float | Velocity error gain factor of velocity PID in the vehicle x direction. |
| controller_velocity_PID | velocity_ki_x | float | Integral factor of velocity PID in the vehicle x direction. |
| controller_velocity_PID | velocity_k_drag_y | float | Feed-forward drag gain factor of velocity PID in the vehicle y direction. |
| controller_velocity_PID | velocity_k_error_y | float | Velocity error gain factor of velocity PID in the vehicle y direction. |
| controller_velocity_PID | velocity_ki_y | float | Integral gain factor of velocity PID in the vehicle y direction. |

### cmd_generator.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| controller_motor_cmd_generator | motor_config | string | Name of the motor configuration. Current allowable values "X" or "H". |
