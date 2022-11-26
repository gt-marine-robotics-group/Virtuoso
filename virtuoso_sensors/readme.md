# Virtuoso Sensors

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

### IMU Troubleshooting
A good way to reset the IMU is to use [sensorconnect microstrain](https://www.microstrain.com/software/sensorconnect) to reboot the device or stop it from streaming. However, do not use this software to calibrate the magnetometer. This software calibrates the IMU backwards.

Instead, use the Iron Calibration under MIP monitor (https://www.microstrain.com/software) to calibrate the IMU.

If you are getting an error relating to "cannot connect to /dev/ttyACM1", try using a different number instead of 1 (start at 0). Typically, microROS will be using port 0 and the IMU will be using port 1. However, if microros is not being used then the IMU should use port 0.


