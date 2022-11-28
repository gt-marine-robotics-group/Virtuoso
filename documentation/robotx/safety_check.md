
# Safety Check

## Launch

In your workspace, run the following commands:
```
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to virtuoso_autonomy
source install/setup.bash
ros2 run virtuoso_autonomy robotx_safety_check_setup.launch.py sim_time:=<sim_time> usv:=<usv>
```

In a second terminal, run the following commands:
```
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run virtuoso_autonomy robotx_safety_check.launch.py usv:=<usv>
```

Replace `<usv>` and `<sim_time>` with the appropriate values.

## Walk-through

1. First, the challenge is set up in simulation, as seen below.

![Safety Check in Simulation](/documentation/images/robotx_safety_check1.png)

2. When the autonomy node first starts, it will enable station keeping. Then, it waits to receive two buoys from Virtuoso Perception. When it has received two buoys, it will find the midpoint of them and send a waypoint between the two.

![First path plan](/documentation/images/robotx_safety_check2.png)

3. When the USV has finished navigating through the first waypoint, it will wait to receive two more buoys from Virtuoso Perception. When it has received two new buoys, it will find the midpoint of them and send a waypoint between the two.

![Second path plan](/documentation/images/robotx_safety_check3.png)

4. When the USV has finished navigating through the second gate it will station keep at the destination.

![Final station keeping](/documentation/images/robotx_safety_check4.png)
