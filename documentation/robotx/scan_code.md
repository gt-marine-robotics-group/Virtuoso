# Scan Code

## Launch

In a terminal, run the following commands:
```
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to virtuoso_autonomy
source install/setup.bash
ros2 run virtuoso_autonomy robotx_scan_code_setup.launch.py sim_time:=<sim_time> usv:=<usv>
```

In a second terminal, run the following commands:
```
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run virtuoso_autonomy robotx_scan_code.launch.py usv:=<usv>
```

Replace `<usv>` and `<sim_time>` with the appropriate values.

## Walk-through

1. First, the challenge is set up in simulation, as shown below.

![Scan code simulation](/documentation/images/robotx_scan1.png)

2. When autonomy starts, it enables station keeping and sends a request to the perception server to begin scanning the code.

3. Next, perception determines the location of the code by passing incoming images through a red filter. The largest red rectange found after 5 seconds is assumed to be the location of the code.

![Red filter](/documentation/images/robotx_scan2.png)

4. Then, the perception server identifies the code sequence by continuing to apply color filters to the incoming images.

![Red filter](/documentation/images/robotx_scan2.png)
![Green filter](/documentation/images/robotx_scan3.png)
![Blue filter](/documentation/images/robotx_scan4.png)
