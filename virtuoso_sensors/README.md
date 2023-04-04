# Virtuoso Sensors

## Contents
- [LIDAR](#lidar)
- [Camera](#camera)
- [GPS](#gps)
- [IMU](#imu)
  - [Start Sequence](#start-sequence)
  - [Troubleshooting](#troubleshooting)
- [Virtuoso Nodes](#virtuoso-nodes)
  - [camera_info_node.py](#camera\_info\_nodepy)
  - [lidar_republish.py](#lidar\_republishpy)
  - [laser_to_pcd_node.py](#laser\_to\_pcd\_nodepy)
  - [f9p_gps_republish.py](#f9p\_gps\_republishpy)
  - [gx3_republish.py](#gx3\_republishpy)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [microstrain.yaml](#microstrainyaml)

## LIDAR
Supports velodyne VLP-16 LIDAR (RobotX 2022) and Hokuyo Lidars (Roboboat 2023).

## Camera
Supports any camera that can communicate through USB. Currently using PlayStation Eyes.

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

### camera_info_node.py
Publishes the appropriate [CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html) for each camera. Camera information found in the `camera_config.yaml` file in the config directory of `virtuoso_perception`.

### lidar_republish.py
Republishes the incoming Lidar data in the `wamv/lidar_wamv_link` frame. Additionally adds an intensity field to the data (set to 0 for each point) because AutowareAuto requires an intensity field for its PointCloud processing.

### laser_to_pcd_node.py
Subscribes to a a [LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html) topic and republishes the data as a [PointCloud](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html). Uses functions in `laser_geometry.py` to do the conversion.

### f9p_gps_republish.py
Republishes the incoming GPS data in the correct frame.

### gx3_republish.py
Republishes the incoming IMU data in the correct frame.

## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| prefix/points | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | lidar_link | Used by perception. |
| prefix/image_raw | [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) | Frames specified in `camera_config.yaml` in `virtuoso_perception`. | Used by perception. |
| prefix/camera_info | [sensor_msgs/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html) | Frames specified in `camera_config.yaml` in `virtuoso_perception`. | Used by perception. |
| prefix/gpx/fix | [sensor_msgs/NavSatFix](https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html) | ubx | Used by localization. |
| prefix/imu | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) | imu_link | Used by localization. |

## Parameters

### microstrain.yml
Unfortunately, no documentation found online.

### urg.yml
No documentation specifically for parameters found online, but should be intuitive. Make sure `publish_intensity` is `True` as AutowareAuto requires an intensity field.

