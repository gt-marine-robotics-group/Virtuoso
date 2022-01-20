## How to run localization:

Ensure both rviz and gazebo are running:

roslaunch vrx_gazebo vrx.launch

roslaunch wamv_gazebo rviz_vrx.launch

Start a ros bridge by sourcing both ros1 and ros2, then running:
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

Run the ekf.launch.py launch file.

Odometry data should now be published to /localization/odometry

## Description

Currently, the package uses robot_localization for basic state estimation. It uses a single EKF that filters IMU data (including orientation, linear acceleration, and attitude rates) and GPS data (absolute position only). There is also a navsat transform node that transforms the GPS data into coordinates in the odometry frame utilizable by the EKF. Currently, the continual_ekf node simply transfers the data from the sensors to the robot_localization node. The goal is to expand on this functunality in the future to implement a custom EKF. 

The base link frame is wamv/base_link, while the odom frame serves as the only global frame - there is no separate map frame, as the package is only using IMU and GPS data so there is not enough continous odometry data to warrant a second EKF and separate map frame.
