# Docking

## Launch

In a terminal, run the following commands:
```
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to virtuoso_autonomy
source install/setup.bash
ros2 run virtuoso_autonomy robotx_dock_setup.launch.py sim_time:=<sim_time> usv:=<usv>
```

In a second terminal, run the following commands:
```
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run virtuoso_autonomy robotx_dock.launch.py usv:=<usv>
```

Replace `<usv>` and `<sim_time>` with the appropriate values.

## Walk-through

1. First, the challenge is set up in simulation, as seen below.

![Docking simulation](/documentation/images/robotx_docking1.png)

2. When autonomy starts, station keeping is activated. Then, autononmy will wait until the relative positions of the docks are identified by perception. When the positions have been identified, the USV will translate to some distance in front of the docks.

![Docking approach](/documentation/images/robotx_docking2.png)

3. Once the USV has finished approaching, it will translate to the correct dock. When in front of the correct dock, it will navigate to the midpoint of the dock in front.

![Docking entry](/documentation/images/robotx_docking3.png)

4. Finally, the USV will navigate an additional few meters forward to fully enter the dock.

![Docking full entry](/documentation/images/robotx_docking4.png)

5. The USV will then station keep.

![Docking final station keeping](/documentation/images/robotx_docking5.png)
