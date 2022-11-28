# Gymkhana

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

![Gymkhana simulation](/documentation/images/robotx_gymkhana1.png)

2. When autonomy starts, station keeping is enabled. Then, autonomy waits for perception to send buoys. When autonomy is able to find two tall buoys, it sends a waypoint at the midpoint of the two buoys.

![First navigation](/documentation/images/robotx_gymkhana2.png)

3. Autonomy will continue to attempt to find buoys to navigate through until the USV has navigated through as many channels specified in the `gymkhana.yaml` parameter file.

![Navigation](/documentation/images/robotx_gymkhana3.png)
![Navigation](/documentation/images/robotx_gymkhana4.png)

4. When the USV is done navigating, it will station keep at its final pose.

![Final station keeping](/documentation/images/robotx_gymkhana5.png)
