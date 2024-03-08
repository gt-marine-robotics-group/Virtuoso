# Virtuoso Localization

## Contents
- [Robot Localization](#robot-localization)
- [Virtuoso Nodes](#virtuoso-nodes)
  - [republisher_node.py](#republisher\_nodepy)
  - [localization_debugger_node.py](#localization\_debugger\_nodepy)
  - [waypoint_saver_node.py](#waypoint\_saver\_nodepy)
  - [multi_tasks_waypoint_saver_node.py](#multi\_tasks\_waypoint\_saver\_nodepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [ekf.yaml](#ekfyaml)
  - [multi_tasks_waypoint_saver.yaml](#multi\_tasks\_waypoint\_saveryaml)
  - [sensor_config.yaml](#sensor\_configyaml)
- [A Note on the Waypoint Savers](#a-note-on-the-waypoint-savers)

## Robot Localization

Currently, the package uses robot_localization for basic state estimation. It uses a single EKF that filters IMU data (including orientation, linear acceleration, and attitude rates) and GPS data (absolute position only). There is also a navsat transform node that transforms the GPS data into coordinates in the odometry frame utilizable by the EKF. 

The base link frame is wamv/base_link, while the odom frame serves as the only global frame - there is no separate map frame, as the package is only using IMU and GPS data so there is not enough continous odometry data to warrant a second EKF and separate map frame.

## Virtuoso Nodes

### republisher_node.py
This node publishes the GPS and IMU data to Robot Localization only when it has received data from both the GPS and IMU.

### localization_debugger_node.py
This node takes in the USV's odometry and motor controller commands and publishes relevant information to the terminal, including the USV's position, heading, and calculated forces.

### waypoint_saver_node.py
When run, this node allows the user to save a series of GPS waypoints (with IMU orientation) and store the waypoints in a YAML file. To save a waypoint, press `>>`. To remove a waypoint, press `<<`. When all the waypoints needed are saved, press `@!`. 

The waypoints will be stored in a YAML file in the computer's `~/mrg/waypoints_raw` directory in the format `points_{num}.yaml`. The saver will not override previously saved waypoints, but will instead create a new YAML file with a greater `num` than the other files. The file will have two fields which look like ROS2 parameters, `ll_points` and `orientations`, where both are arrays of the same length. For every waypoint saved, there will be an element in `ll_points` containing the GPS latitude, longitude, and elevation. Additionally, for every waypoint saved, there will be an element in `orientations` containing the x, y, z, and w orientation values.

Please also see the [additional section](#a-note-on-the-waypoint-savers) on the waypoint players.

### multi_tasks_waypoint_saver_node.py
When run, this node allows the user to save a series of GPS waypoints (with IMU orientation) for separate tasks and store the waypoints in a YAML file. To save a waypoint for a task, press `>>`. To remove a waypoint for a task, press `<<`. To change task number, use `++` to increment task number and `--` to decrement task number. When all waypoints needed are saved, press `@!`.

The waypoints will be stored in a YAML file in the computer's `~/mrg/semis_waypoints` directory in the format `points_{num}.yaml`, similar to the basic waypoint saver. For every task, there will be an `ll_points_{task_num}` field and an `orientations_{task_num}` field. Each field will store the waypoint data in the same way the basic waypoint player does.

Please also see the [additional section](#a-note-on-the-waypoint-savers) on the waypoint players.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| usv/gps/fix | [sensor_msgs/NavSatFix](https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html) | ubx | Used by EKF. |
| usv/imu | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) | imu_link | Used by EKF. |
| /gps/filtered | [sensor_msgs/NavSatFix](https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html) | ubx | Used by all waypoint savers. |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /localization/odometry | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | odom | Used for navigation. |

## Parameters

### ekf.yaml
Documentation for the parameters can be found [here](http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html).

### multi_tasks_waypoint_saver.yaml

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| localization_multi_task_waypoint_saver | num_tasks | int | Number of tasks for which to save waypoints for. |

### sensor_config.yaml

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| localization_republisher | imu_topic | string | Topic to receive raw IMU data. |
| localization_republisher | gps_topic | string | Topic to receive raw GPS data. |
| localization_republisher | gps_vel_topic | string | Topic to receive raw GPS velocity data. |

## A Note on the Waypoint Savers

The waypoint savers, both the standard saver and the multi task saver, use the python library `pynput`. There are two main points to consider when using the savers:

1. When saving waypoints with the physical USV, the saver should be run from the laptop on shore, not the computer onboard the USV. This is becuase with `pynput` by default we cannot detect keystrokes over ssh. As long as the on shore laptop is on the same wifi and ROS_DOMAIN_ID as the onboard computer, the necessary data to save waypoints should be sent by ROS 2 over wifi. If not, you will see a message on the terminal when trying to save a waypoint. Once the waypoints have been saved on the on shore laptop, the yaml file can be sent to the onboard computer over the `scp` protocol. For example, `scp ~/mrg/waypoints_raw/points_10.yaml mrg@192.168.1.200:~/mrg/waypoints_raw`.

2. As of Ubuntu 22, Ubuntu uses the Wayland windowing system. On Ubuntu 20, `pynput` was able to detect keystrokes sent while in the standard terminal. However, this is not the case on Ubuntu 22, and we have not yet found a fix for this. However, the keystrokes will be detected on Ubuntu 22 if the keystrokes are done in the VSCode terminal. Therefore, we recommend running the waypoint savers in a VSCode terminal.