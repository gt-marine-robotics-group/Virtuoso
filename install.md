## Install ROS 2 Foxy:

Note: This tutorial assumes you are using Ubuntu Linux

https://docs.ros.org/en/foxy/Installation.html

Note: Do not add sourcing Foxy to your shell startup script as you will need to source ROS 1 in other terminals to run the VRX simulation

## Install VRX Environment

https://github.com/osrf/vrx/wiki/system_setup_tutorials

## Install Virtuoso

You can now clone the virtuoso repo:

https://github.com/gt-marine-robotics-group/Virtuoso

In order to launch the X-configuration thrusters WAMV with sensors that we will be using, you will need to build the wamv urdf file:

https://github.com/osrf/vrx/wiki/tutorials-Creating%20a%20custom%20WAM-V%20Thruster%20and%20Component%20Configuration%20For%20Competition


The thruster and component config yaml files we are currently using are located here:

https://github.com/gt-marine-robotics-group/Virtuoso/tree/controller/utils/wamv_X


### Build the virtuoso packages you will be running. Be sure to source foxy and be in the root of your dev workspace:

Ex:
```
colcon build --packages-select virtuoso_localization
colcon build --packages-up-to virtuoso_autonomy
```
### Install robot localization:
You can also install any other missing ROS dependencies using similar syntax
```
sudo apt install ros-foxy-robot-localization
```
### Install other dependencies
Install processing dependencies (https://github.com/gt-marine-robotics-group/Virtuoso/tree/main/virtuoso_processing)

Install perception dependencies (https://github.com/gt-marine-robotics-group/Virtuoso/tree/main/virtuoso_perception)

### Clone the messages workspace into a separate workspace from the vrx workspace and the workspace with virtuoso
https://github.com/gt-marine-robotics-group/Virtuoso-Messages

### Clone and build the ros1 to ros2 bridge
https://github.com/gt-marine-robotics-group/Virtuoso-Messages/tree/main

## You should now be able to launch the gazebo simulation:
You must also source the messages workspace that you built earlier as well.
```
source /opt/ros/noetic/setup.bash
source  ~/vrx_ws/devel/setup.bash
roslaunch vrx_gazebo sydneyregatta.launch urdf:=$HOME/my_wamv/my_wamv.urdf
```
Run rviz in another terminal, this publishes the state transforms between components and the base_link frame:
```
source /opt/ros/noetic/setup.bash
source  ~/vrx_ws/devel/setup.bash
roslaunch wamv_gazebo rviz_vrx.launch
```
In order to get the ROS 2 virtuoso code to communicate with gazebo, you will need to create a bridge between ROS1 and ROS2, as follows:
You must also source the vrx workspace, the virtuoso msgs workspace, and the bridge workspace.

```
source /opt/ros/foxy/setup.bash
source /opt/ros/noetic/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
You can now launch the Virtuoso packages as you desire. Be sure to source the Virtuoso workspace in your terminal by running `. install/setup.bash` or similar while in the root of the workspace.
```
ros2 launch virtuoso_autonomy main.launch.py
```

## Installation of Sensor Drivers
### IMU
See https://github.com/LORD-MicroStrain/microstrain_inertial/tree/ros2
```
sudo apt-get update && sudo apt-get install ros-foxy-microstrain-inertial-driver
sudo apt-get update && sudo apt-get install ros-foxy-microstrain-inertial-rqt
```
