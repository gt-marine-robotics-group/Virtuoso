# Enter and Exit

## Launch

In a terminal, run the following commands:
```
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to virtuoso_autonomy
source install/setup.bash
ros2 run virtuoso_autonomy robotx_enter_exit_setup.launch.py sim_time:=<sim_time> usv:=<usv>
```

In a second terminal, run the following commands:
```
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run virtuoso_autonomy robotx_enter_exit.launch.py usv:=<usv>
```

Replace `<usv>` and `<sim_time>` with the appropriate values.

## Walk-through

1. First, the challenge is set up in simulation, as seen below.

![Enter exit simulation](/documentation/images/robotx_enter_exit1.png)

2. When autonomy first starts, it will enable station keeping. It will then wait for Virtuoso Perception to send it buoys. When it has received at least three buoys, it will identify the location of the 3 entrances. Then, it will choose a random entrance and send a waypoint at the midpoint of the entrance.

![Path through midpoint of first gate](/documentation/images/robotx_enter_exit2.png)

3. When the USV has navigated to the midpoint, it will then determine where the buoy it needs to loop is located. When it has found the buoy, it will send a series of waypoints to the navigation server that loop the buoy and end at the USV's current pose.

![Navigating around buoy](/documentation/images/robotx_enter_exit3.png)
![Navigating around buoy](/documentation/images/robotx_enter_exit4.png)
![Navigating around buoy](/documentation/images/robotx_enter_exit5.png)
![Navigating around buoy](/documentation/images/robotx_enter_exit6.png)
![Navigating around buoy](/documentation/images/robotx_enter_exit7.png)

4. When the USV has arrived back to the entrance gate, it will station keep there.

![Final station keeping](/documentation/images/robotx_enter_exit8.png)
