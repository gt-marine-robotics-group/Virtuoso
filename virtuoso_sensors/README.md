# Virtuoso Sensors

## Contents
- [LIDAR](#lidar)
- [Camera](#camera)
- [GPS](#gps)
- [IMU](#imu)
  - [Accounting for Magnetic Declination](#accounting-for-magnetic-declination)
  - [Calibration](#calibration)
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

### Accounting for Magnetic Declination

The vehicle is built around the principle that the map frame has east-north-up coordinate axes. This simplifies debugging significantly and makes it easier to specify absolute orientations when planning GPS waypoints. However, the magnetometer in the IMU will naturally cause it to output a heading in the magnetic ENU frame rather than the actual ENU frame. This means that the magnetic declination, or angle between the magnetic and true north directions, needs to be accounted for. This is done in the actual transformation for the IMU frame specified, at the moment, in gps.launch.py. While you may rotate this frame to account for the IMU's orientation with respect to the robot, it must be additionally rotated in the global z axis (up) by the magnitude of the magnetic declination in the local area. This can be found online. There are also useful phone apps, like GPS status for android, that display this information. The direction is theoretically the direction such that when the vehicle is oriented towards magnetic north, the IMU frame is oriented such that it is pointing towards true north (if the IMU thinks it's pointed at true north, the vehicle is in reality pointed at magnetic north). Use a phone compass that has built in correction for magnetic declination (again, such as GPS status) to check that the boat knows it's in the correct orientation. If it seems to be off, try reversing the rotation direction and see if that improves things.

### Calibration

Use the Iron Calibration under MIP monitor (https://www.microstrain.com/software) to calibrate the IMU. Ideally, the vehicle should be completely assembled, with all batteries present and the vehicle powered on. 

1. Disconnect the IMU from the vehicle and attach it to your computer. Ideally, you should keep your computer as far away from the IMU as possible to keep the magnetic field of your computer from interfering with the calibration. 
2. Next, ensure your IMU shows up in the device list and select it. You should be able to view the current calibration. 
3. Click on the red record button to begin recording. 
4. Spin the vehicle in a 360 circle at a moderate pace. You should see magnetic field readings appear in a perfect circle on your screen, ideally. 
5. Switch to the spherical fit tab and write the calibration to the device. 
6. Disconnect and reconnect to the device and ensure your calibration was written. 
7. If you wish to be absolutely sure, rerun the calibration sequence and ensure that your next calibration is somewhat close to your first calibration. The "z" offset of your calibration may differ significantly, this will not affect the calibration. 
8. Reconnect the device to the computer.

### Start Sequence
The IMU requires a sequence to correctly start.

- Run `ros2 launch virtuoso_sensors imu.launch.py first_start:=1`
- You should seen an error along the lines of "imu does not support the declination command. transition failed"
- Do not pass a first_start launch argument for any future IMU launches

### Troubleshooting
A good way to reset the IMU is to use [sensorconnect microstrain](https://www.microstrain.com/software/sensorconnect) to reboot the device or stop it from streaming. However, do not use this software to calibrate the magnetometer. This software calibrates the IMU backwards. The legacy inertial products support software is required.

Instead, use the Iron Calibration under MIP monitor (https://www.microstrain.com/software) to calibrate the IMU.

If you switch to a different physical IMU unit, even of the same model, you may have to change the port name. List the files present in /dev/serial/by-id. One should obviously be a microstrain device. Use this file name as the port name in /config/microstrain.yml. 

Outdated:
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

