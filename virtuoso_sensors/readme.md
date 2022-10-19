# Virtuoso Sensors

For ZED-F9P-00B-002 GPS and 3DM-GX3 -25 IMU.

## Install Dependencies
Microstrain: Feel free to install using buildfarm

https://github.com/LORD-MicroStrain/microstrain_inertial/tree/ros2

Ublox dgnss: You will have to clone this one and build it. Be sure to create the udev rule as directed in the package readme.

https://github.com/aussierobots/ublox_dgnss

## IMU Start Sequence
The IMU requires a sequence to correctly start.

<ol>
  <li>Launch imu.launch.py. No errors should be indicated, but nothing will be published to the imu topics.</li>
  <li>Stop imu.launch.py</li>
  <li>Go to /config/microstrain.yml, comment out the line reading "filter_sensor2vehicle_frame_selector: 0"</li>
  <li>Rebuild the package (colcon build ...) and relaunch imu.launch.py</li>
  <li>You should seen an error along the lines of "imu does not support the declination command. transition failed"</li>
  <li> Remove the comment in microstrain.yml to reactivate the parameter</li>
  <li>Rebuild and relaunch imu.launch.py. The imu should now be started correctly</li>
  </ol>

## IMU Troubleshooting:
A good way to reset the IMU is to use sensorconnect microstrain (https://www.microstrain.com/software/sensorconnect) to reboot the device or stop it from streaming. However, do not use this software to calibrate the magnetometer. This software calibrates the IMU backwards.

Instead, use the Iron Calibration under MIP monitor (https://www.microstrain.com/software) to calibrate the IMU.

If you are getting an error relating to "cannot connect to /dev/ttyACM1", try using a different number instead of 1 (start at 0). Typically, microROS will be using port 0 and the IMU will be using port 1. However, if microros is not being used then the IMU should use port 0.

## GPS Output on Windows
This can be used to check GPS signal and fix (https://www.u-blox.com/en/product/u-center)
