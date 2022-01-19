How to run localization:

Ensure both rviz and gazebo are running:
roslaunch vrx_gazebo vrx.launch
roslaunch wamv_gazebo rviz_vrx.launch

Start a ros bridge by sourcing both ros1 and ros2, then running:
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

Run the launch_ekf.launch.py launch file.

Odometry data should now be published to /localization/odometry
