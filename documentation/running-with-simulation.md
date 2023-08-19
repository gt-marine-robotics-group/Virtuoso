# Running Virtuoso in Simulation

This document assumes you have the file structure below following installation:
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
|       └───ublox_dgnss
```
If you do not have this structure, you will need to modify some commands.

## 0. Create a Custom WAM-V urdf

Note: Starting with RoboBoat, we are now providing the already built URDF in `utils/<competition>`. You can use this URDF in later steps (just make sure to use the appropriate file path) and ignore the remainder of this step.

This is a ONE TIME step. 

Inside of `Virtuoso/utils/wamv_X` there are yaml files containing the configuration of our motors and sensors on the WAM-V in simulation. For VRX to recognize this is the configuration we want our WAM-V to have, we need to create a custom urdf file from these configuration yaml files. To do that, run the following commands: 

```
source /opt/ros/humble/setup.bash
source ~/mrg/vrx_ws/install/setup.bash
ros2 launch vrx_gazebo generate_wamv.launch thruster_yaml:=$HOME/mrg/dev_ws/src/Virtuoso/utils/wamv_X/thruster_config.yaml  component_yaml:=$HOME/mrg/dev_ws/src/Virtuoso/utils/wamv_X/component_config.yaml wamv_target:=$HOME/mrg/wamv.urdf
```

You should see a `wamv.urdf` file inside your `mrg` directory when the process finishes.

## 1. Launching VRX

In a terminal, run the following commands:

```
source /opt/ros/humble/setup.bash
source ~/mrg/vrx_ws/install/setup.bash
ros2 launch vrx_gz competition.launch.py urdf:=$HOME/mrg/wamv.urdf
```

You should see the following come up:

![Simulation](/documentation/images/rws1.png)

If you look under the WAM-V, you should see the holonomic motor configuration:

![Holonomic config](/documentation/images/rws2.png)

You can also drag objects into Gazebo to set up a challenge. Below, for example, is a set up for the safety check:

![Safety check](/documentation/images/rws3.png)

## 4. Run a Setup Script

To run the setup script for a task, follow the below template:

```
source /opt/ros/humble/setup.bash
cd ~/mrg/dev_ws
colcon build --packages-up-to virtuoso_autonomy
source install/setup.bash
ros2 launch virtuoso_autonomy <competition>_<task_name>_setup.launch.py usv:=<arg>
```

See the documentation on the [launch arguments](/documentation/launch-arguments.md).

To run the setup script for the RoboBoat Safety Check task in simulation, launch the following:

```
ros2 launch virtuoso_autonomy roboboat_safety_check_setup.launch.py usv:=vrx_roboboat
```

You should see an RVIZ appear similar to the image below (we have updated our RVIZ launch so it will look a bit different):

![Virtuoso RVIZ](/documentation/images/rws5.png)

## 5. Run the Autonomy Script

To run the autonomy script for a task, follow the below template:

```
source /opt/ros/foxy/setup.bash
cd ~/mrg/dev_ws
source install/setup.bash
ros2 launch virtuoso_autonomy <competition>_<task_name>.launch.py usv:=<arg>
```

To run the autonomy script for the RoboBoat Safety Check task in simulation, launch the following:

```
ros2 launch virtuoso_autonomy roboboat_safety_check.launch.py usv:=vrx_robotx
```

You should see the robot begin to path plan and move in RVIZ and Gazebo:

![Virtuoso Running](/documentation/images/rws6.png)
