# Virtuoso Sensors

## Contents
- [LIDAR](#lidar)
- [Camera](#camera)
- [GPS](#gps)
- [IMU](#imu)
  - [Start Sequence](#start-sequence)
  - [Troubleshooting](#troubleshooting)
- [Virtuoso Nodes](#virtuoso-nodes)
  - [lidar_republish.py](#lidar\_republishpy)
  - [camera_republish.py](#camera\_republishpy)
  - [f9p_gps_republish.py](#f9p\_gps\_republishpy)
  - [gx3_republish.py](#gx3\_republishpy)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [microstrain.yaml](#microstrainyaml)

## LIDAR
Supports velodyne VLP-16 LIDAR.

## Camera
Supports any camera that can communicate through USB.

## GPS
Supports ZED-F9P-00B-002 GPS.

[This](https://www.u-blox.com/en/product/u-center) can be used to check GPS signal and fix.

## IMU
Supports 3DM-GX3 -25 IMU.

### Start Sequence
The IMU requires a sequence to correctly start.

- Run `ros2 launch virtuoso_sensors imu.launch.py first_start:=1`
- You should seen an error along the lines of "imu does not support the declination command. transition failed"
- Do not pass a first_start launch argument for any future IMU launches

### Troubleshooting
A good way to reset the IMU is to use [sensorconnect microstrain](https://www.microstrain.com/software/sensorconnect) to reboot the device or stop it from streaming. However, do not use this software to calibrate the magnetometer. This software calibrates the IMU backwards.

Instead, use the Iron Calibration under MIP monitor (https://www.microstrain.com/software) to calibrate the IMU.

If you are getting an error relating to "cannot connect to /dev/ttyACM1", try using a different number instead of 1 (start at 0). Typically, microROS will be using port 0 and the IMU will be using port 1. However, if microros is not being used then the IMU should use port 0.

## Virtuoso Nodes

### lidar_republish.py
Republishes the incoming LIDAR data in the correct frame.

### camera_republish.py
Republishes the incoming camera data in the correct frame.

### f9p_gps_republish.py
Republishes the incoming GPS data in the correct frame.

### gx3_republish.py
Republishes the incoming IMU data in the correct frame.

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| usv/lidar_points | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | Used by processing. |
| usv/camera_image | [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) | camera_link | Used by processing. |
| usv/gpx/fix | [sensor_msgs/NavSatFix](https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html) | ubx | Used by localization. |
| usv/imu | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) | imu_link | Used by localization. |

## Parameters

### microstrain.yaml
Unfortunately, no documentation found online.

