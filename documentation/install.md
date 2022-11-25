## Recommended File Structure (following installation)
```
~/openvdb
~/mrg
└───vrx_ws 
└───dev_ws
│   └───build (auto-generated during a build)
│   └───install (auto-generated during a build)
│   └───log (auto-generated during a build)
│   └───src
│       └───Virtuoso
│       └───AutowareAuto
│       └───spatio_temporal_voxel_layer
|       └───ublox_dgnss
|       └───velodyne
```
Having all installed directories within a directory (e.g. mrg) is optional, but it can help to keep everything organized. It is strongly recommended that you install Virtuoso, AutowareAuto, and all other ROS2 dependencies being built from source within one workspace (e.g. dev_ws). This makes building Virtuoso simpler.

## Install Ubuntu 20.04
To get Ubuntu 20.04, we reccommend using a virtual machine (unless you can already boot into Ubuntu 20.04 from your machine). [This tutorial](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview) explains how to install Ubuntu on VirtualBox.

**Note:** You will probably want to allocate at least around 8 GB of memory to the VM. Lower memory allocation can make the simulation environment (Gazebo) difficult to run, and Gazebo may crash. If you cannot allocate enough memory for your VM, we have computers in the lab you can use to test software in simulation.

## Set up an SSH key for GitHub authentication
Follow [this guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh) for adding an ssh key to your github account. Make sure you generate the ssh key on your VM.

## Install ROS2 Foxy
Follow [this installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) to install Foxy. Make sure you do this within your VM.

**Note:** Do not add the command to source Foxy to your ~/.bashrc. This is not in the installation guide, but it is in the tutorials. Because we need to use ROS Noetic for our simulation, we must choose which ROS distribution to source every time we open a terminal.

## Install ROS Noetic
Follow [this installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu) to install Noetic. Make sure you do this within your VM.

**Note:** Do not add the command to source Noetic to your ~/.bashrc. This is step 1.5 of the guide. Please skip this step.

## Install VRX
Follow [this installation guide](https://github.com/osrf/vrx/wiki/system_setup_tutorials) to install VRX. For step 1, select Option A: Configure your host machine. 

Before running any commands in a terminal for this installation, you need to source ROS Noetic:
```
source /opt/ros/noetic/setup.bash
```

## Install ROS1 Bridge
In a new terminal, run the following command to install ros1_bridge:
```
sudo apt install ros-foxy-ros1-bridge
```

Because our software is written in ROS2 Foxy and the simulation is written in ROS Noetic, we need a ROS bridge running in order to receive data from and send data to the simulation.

## Install Virtuoso
Within your workspace, run the following command:
```
git clone git@github.com:gt-marine-robotics-group/Virtuoso.git
```

## Install OpenVDB
OpenVDB is an open-source software needed to run Spatio-Temporal Voxel Layer (another open-source package we are using). To install it, run the following commands:
```
sudo apt-get install -y libboost-iostreams-dev
sudo apt-get install -y libtbb-dev
sudo apt-get install -y libblosc-dev
```
```
git clone git@github.com:AcademySoftwareFoundation/openvdb.git
cd openvdb
mkdir build
cd build
cmake ..
sudo make -j4
sudo make install
```
More detailed documentation about OpenVDB can be found on [their GitHub](https://github.com/AcademySoftwareFoundation/openvdb).

## Install Spatio-Temporal Voxel Layer
Within your workspace, run the following command:
```
git clone -b foxy-devel https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git
```

## Install AutowareAuto
Within your workspace, follow the [guide](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-no-ade.html) to install AutowareAuto. Make sure to replace $ROS_DISTRO with foxy.

## Install Ublox-Dgnss
Within your workspace, run the following command:
```
git clone https://github.com/aussierobots/ublox_dgnss
```

## Install Velodyne
Within your workspace, run the following command:
```
git clone -b foxy-devel https://github.com/ros-drivers/velodyne.git
```

## Remove conflicting packages
Both AutowareAuto and Velodyne have certain overlapping package names. From the AutowareAuto directory, run the following commands:
```
cd src
cd drivers
rm -rf velodyne_driver velodyne_nodes
```
Now, when our velodyne driver is built, it will use the driver from velodyne not from AutowareAuto.
