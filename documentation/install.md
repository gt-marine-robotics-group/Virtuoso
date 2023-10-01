# Installation

## Recommended File Structure (following installation)
```
~/mrg
└───vrx_ws 
└───dev_ws
│   └───build (auto-generated during a build)
│   └───install (auto-generated during a build)
│   └───log (auto-generated during a build)
│   └───src
│       └───Virtuoso
|       └───ublox_dgnss
|       └───urg_node
```
Having all installed directories within a directory (e.g. mrg) is optional, but it can help to keep everything organized. It is strongly recommended that you install Virtuoso and all other ROS2 dependencies being built from source within one workspace (e.g. dev_ws). This makes building Virtuoso simpler.

## 1. Install Ubuntu 22.04
To get Ubuntu 22.04, we reccommend using a virtual machine (unless you can already boot into Ubuntu 20.04 from your machine). [This tutorial](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview) explains how to install Ubuntu on VirtualBox.

**Note:** You will probably want to allocate at least around 8 GB of memory to the VM. Lower memory allocation can make the simulation environment (Gazebo) difficult to run, and Gazebo may crash. If you cannot allocate enough memory for your VM, we have computers in the lab you can use to test software in simulation. Additionally, you can dual boot your computer with Ubuntu to take advantage of all memory.

## 2. Set up an SSH key for GitHub authentication
Follow [this guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh) for adding an ssh key to your github account. Make sure you generate the ssh key on your VM.

## 3. Install ROS 2 Humble
Follow [this installation guide](https://docs.ros.org/en/humble/Installation.html) to install Humble. Make sure you do this within your VM.

## 4. Install VRX
Follow [this installation guide](https://github.com/osrf/vrx/wiki/getting_started_tutorial) to install VRX. For step 1, select Option A: Configure your host machine. 

## 5. Install Virtuoso
Within your workspace (e.g. dev_ws/src), run the following command:
```
git clone git@github.com:gt-marine-robotics-group/Virtuoso.git
```

## 6. Install Ublox-Dgnss
Note: you can skip this step if you are not using a physical sensor.

Within your workspace (e.g. dev_ws/src), run the following command:
```
git clone https://github.com/aussierobots/ublox_dgnss
```

You may also need to create a udev rule as directed in their [README.md](https://github.com/aussierobots/ublox_dgnss/blob/main/README.md).


Note that this package is only used for communicating with the physical IMU we have. It is not needed for simulation.

## 7. Install urg_node
Note: you can skip this step if you are not using a physical sensor.

Within your workspace (e.g. dev_ws/src), run the following command:
```
git clone -b ros2-devel https://github.com/ros-drivers/urg_node.git
```

Note that this package is only used for communicating with the physical Hokuyo lidar we have. It is not needed
for simulation.


## 8. Installing Remaining dependinces
From the root of your workspace (e.g dev_ws), run the following commands:
```
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/Virtuoso --ignore-src -r
```
This will install all of Virtuoso's dependencies that we are not building from source. You may see some packages
were not found; these are likely the sensor dependencies that were skipped earlier.

## 9. Build Virtuoso
From the root of your workspace, run the following commands:
```
source /opt/ros/humble/setup.bash
```

If you are not using physical sensors, you should then run 
```
colcon build --packages-up-to virtuoso_autonomy --packages-ignore virtuoso_sensors
```

Else, run
```
colcon build --packages-up-to virtuoso_autonomy
```
