# Virtuoso Localization

## Contents
- [Robot Localization](#robot-localization)
- [Virtuoso Nodes](#virtuoso-nodes)
  - [republisher_node.py](#republisher\_nodepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [ekf.yaml](#ekfyaml)

## Robot Localization

Currently, the package uses robot_localization for basic state estimation. It uses a single EKF that filters IMU data (including orientation, linear acceleration, and attitude rates) and GPS data (absolute position only). There is also a navsat transform node that transforms the GPS data into coordinates in the odometry frame utilizable by the EKF. 

The base link frame is wamv/base_link, while the odom frame serves as the only global frame - there is no separate map frame, as the package is only using IMU and GPS data so there is not enough continous odometry data to warrant a second EKF and separate map frame.

## Virtuoso Nodes

### republisher_node.py
This node publishes the GPS and IMU data to Robot Localization only when it has received data from both the GPS and IMU.

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| usv/gps/fix | [sensor_msgs/NavSatFix](https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html) | ubx | Used by EKF. |
| usv/imu | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) | imu_link | Used by EKF. |

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /localization/odometry | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | odom | Used for navigation. |

## Parameters

### ekf.yaml
Documentation for the parameters can be found [here](http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html).
