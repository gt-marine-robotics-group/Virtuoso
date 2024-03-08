# Running on the USV

Running Virtuoso on the USV requires an on-board and ground station computer. The on-board computer is the computer actually running Virtuoso and handles communication between Virtuoso and motors (discussed below). The ground station computer is operated by a human, and, over a wifi network, tells the on-board computer what software to execute.

## 1. Establishing Communication

The on-board computer should be connected to a wifi router on the USV and its IP address should be known. Ideally, the IP address will be statically configured; for most devices in the lab, the IP address over ethernet is set to `192.168.1.200` with the subnet being `192.168.1.x`.

Once a wifi network has been set up and the on-board computer is connected, the ground station computer should be connected to the network. Make sure the ground station computer is on the same subnet as the on-board computer.

To access the terminal of the on-board computer from the ground station computer, run
```
ssh mrg@192.168.1.200
```
and enter the appropriate credentials. 

If using the Docker install on the on-board computer, make sure to be in the container before starting the next steps.

## 2. Establishing Firmware Connection

Virtuoso communicates with the firmware over Micro-Ros. If not using the Docker install, ensure [Micro-Ros is installed](https://gt-mrg.notion.site/micro-ROS-Setup-f94b4cb11ab44c49a2d0658531e895c1), [Bag-of-Tricks is cloned](https://github.com/gt-marine-robotics-group/bag-of-tricks), and `tmux` is installed.

There is a tmuxp yaml config file found in Bag-of-Tricks which will launch a 6-pane tmux session. One of the panes will automatically try to establish a Micro-Ros connection with the main motor control micro-controller and the auxiliary system micro-controller. To start this tmux session, run
```
tmuxp load <path to bag-of-tricks>/tmux/roglan_01.yaml
```
Note that as micro-controllers on the USV are changed, the device ids specified in the config file will need to be updated.

When a Micro-Ros connection is established, the terminal pane will state the subscribers created.

## 3. Launching Sensors

The sensors drivers are launched seperately from other parts of Virtuoso, since sometimes annoying debug messages will be printed to the terminal. From one of the terminal panes, the sensors can be launched by running
```
ros2 launch virtuoso_sensors main.launch.py usv:=<roboboat/robotx>
```
Sensors can also be launched independently in separate launch files if necessary for debugging.

## 4. Launching Autonomy

This process is similar that of running in simulation. In one pane, the setup script for a task should be launched, and in another pane, the autonomy node should be launched.

Note that when passing a `usv` arg of `roboboat` or `robotx` (which should be passed in this context), RVIZ will not be launched. To use RVIZ, you should launch it from the ground station computer. As long as the computer is on the same wifi network and `ROS_DOMAIN_ID` as the on-board computer, and there are no firewall issues, you should be able to subscribe to all necessary topics. On Ubuntu, you can disable the firewall by running `sudo ufw disable`.

## 5. Editing Files

Oftentimes during lake tests, simple bugs will need to be fixed or parameters in yaml files will need to be changed. One option is to use a terminal based text editor like Vim. Another option is to use the ssh extension on VSCode to edit files on the on-board computer using VSCode (note this is not possible when using the Docker container on-board, I think... if someone knows how, please document!).

