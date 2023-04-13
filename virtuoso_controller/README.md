# Virtuoso Controller

## Contents
- [Virtuoso Nodes](#virtuoso-nodes)
  - [choose_pid_node.py](#choose\_pid\_nodepy)
  - [cmd_vel_generator_node.py](#cmd\_vel\_generator\_nodepy)
  - [velocity_pid_node.py](#velocity\_pid\_nodepy)
  - [basic_pid_node.py](#basic\_pid\_nodepy)
  - [motor_cmd_generator_node.py](#motor\_cmd\_generator\_nodepy)
- [External Subscribed Topics](#external-subscribed-topics)
- [External Published Topics](#external-published-topics)
- [Parameters](#parameters)
  - [pidgains.yaml](#pidgainsyaml)

## Virtuoso Nodes

### choose_pid_node.py
This node sends the appropriate waypoints to the Basic PID, and it also decides whether to use the velocity PID for translational movement or the Basic PID.

### cmd_vel_generator_node.py

The outer loop takes the vehicle position and target path as its inputs. It calculates a desired velocity as a sum of two vectors. The first of these is a vector directly towards the line between the closest point on the path and the next point on the path. The second is the vector from the closest point on the path to the next point on the path. These vectors are scaled such that if the vehicle is further from the path, the vector towards the path is favored. In addition, the velocity magnitude is decreased if the vehicle is pointing away from the target velocity so that the vehicle can rotate itself to face in the direction of travel. This is preferred since the hydrodynamics of the vehicle favor motion in the surge direction. 

### velocity_pid_node.py

The main inner loop PID controller takes in the vehicle’s current velocity, target velocity, and target attitude and outputs a goal X and Y force as well as torque. 

The target attitude is the direction such that the vehicle’s +x axis is parallel to the desired velocity until the vehicle comes within a specified distance of its target, at which point the goal attitude specified by navigation package is set as the target. 

### basic_pid_node.py

The last-meter PID controller takes in vehicle velocity, position, and attitude as well as target position and attitude and outputs X and Y force as well as torque commands. The last-meter PID controller is implemented so that station keeping can be tuned separately from the vehicle behavior between goals. 

### motor_cmd_generator_node.py

The control mixer is what sends individual thruster commands based on target X and Y force as well as torque. The rear thruster commands are scaled down in magnitude to not induce excessive torque with Y force commands. In addition, the commands to all thrusters are scaled so that the maximum magnitude is 1.0, but the ratios of all thruster commands are kept. Can choose between the X and H drive configurations. In the X drive configuration, the side motors will output to 0. 

## External Subscribed Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| /localization/odometry | [sensor_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) | odom | Used by choose_pid_node, basic_pid_node, cmd_vel_generator_node, and velocity_pid_node. |
| /navigation/plan | [nav_msgs/Path](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html) | map | Used by choose_pid_node. |
| /controller/is_translation | [std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html) | N/A | If true, vehicle holds its final orientation while moving instead of pointing in the direction of travel. Affects choose_pid and cmd_vel_generator|


## External Published Topics

| Topic | Message Type | Frame | Purpose |
|-------|--------------|-------|---------|
| usv motors | [std_msgs/Float32](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html) | N/A | Thrust for each motor on the USV. |

## Parameters

### pidgains.yaml

Note that the PID gain parameters are currently multiplied by the baked-in PID gains in the PID nodes to get the actual PID gains used. So a 0.5 in the config file would result in half the built-in gain.

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| controller_basic_PID | basic_kp_x | float | Proportional factor of basic PID in the vehicle x direction. |
| controller_basic_PID | basic_kd_x | float | Derivate factor of basic PID in the vehicle x direction. |
| controller_basic_PID | basic_ki_x | float | Integral factor of basic PID in the vehicle x direction. |
| controller_basic_PID | basic_kp_y | float | Proportional factor of basic PID in the vehicle y direction. |
| controller_basic_PID | basic_kd_y | float | Derivate factor of basic PID in the vehicle y direction. |
| controller_basic_PID | basic_ki_y | float | Integral factor of PID in the vehicle y direction. |
| controller_basic_PID | basic_rotate_kp | float | Proportional gain factor of basic PID in rotation. |
| controller_basic_PID | basic_rotate_kd | float | Derivative gain factor of basic PID in rotation. |
| controller_basic_PID | basic_rotate_ki | float | Integral gain factor of basic PID in rotation. |
| controller_velocity_PID | velocity_k_drag_x | float | Feed-forward drag gain factor of velocity PID in the vehicle x direction. |
| controller_velocity_PID | velocity_k_error_x | float | Velocity error gain factor of velocity PID in the vehicle x direction. |
| controller_velocity_PID | velocity_ki_x | float | Integral factor of velocity PID in the vehicle x direction. |
| controller_velocity_PID | velocity_k_drag_y | float | Feed-forward drag gain factor of velocity PID in the vehicle y direction. |
| controller_velocity_PID | velocity_k_error_y | float | Velocity error gain factor of velocity PID in the vehicle y direction. |
| controller_velocity_PID | velocity_ki_y | float | Integral gain factor of velocity PID in the vehicle y direction. |

### cmd_generator.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| controller_motor_cmd_generator | motor_config | string | Name of the motor configuration. Current allowable values "X" or "H". |

## Tuning the Controller

The controller has several layers to it, so tuning it requires a decent understanding of what's going on underneath the hood. Luckily, the dynamics of surface vehicles are quite forgiving, so a huge amount of tuning isn't required.

### Motor Command Generator

Let's first examine the motor command generator. The goal is to create forces and torques on the vehicle from motor commands and differential thrust. A simple free body diagram of the vehicle and its motors should give an intuitive understanding of how this works. In the "H" drive configuration, it is the job of the mechanical team to align the side motors with the vehicle's center-of-mass so that side forces do not induce torques on the vehicle. Of course, as the controller subject matter expert you should be pushing for this. In the "X" drive configuration, having the center of mass offset in the x-direction from the center of the motors can also lead to torques when side forces are commanded. This is solved by reducing the strength of either the forward or rear motors for y-force commands, thus creating a torque balance. It is your job to figure out which set of thrusters to reduce in strength and how much to reduce them by. Again, a free body diagram might help.

### Basic PID

The Basic PID is the simplest controller to tune since when the vehicle is within 2 m of the target it is only a single layer of control (outside of the motor command generator). If the vehicle is not getting to the target point fast enough, increase the proportional gain or decrease the derivative gain. If the vehicle is overshooting or oscillating significantly around the target point, increase the derivative gain or decrease the proportional gain. If strong currents are affecting the vehicle, likely increase the proportional gain. The integral gain can theoretically help with currents over a long period of time but it takes more time to tune. I'd just increase the proportional and derivative gains until you are comfortable with the behavior. It is useful to look at a graph of the vehicle position over time as well as the output of the controller (target force) to ensure it "looks" right. 

### Velocity PID and Command Velocity Generator

The controller is significantly more complex when using the velocity PID since it has to follow the path follower as well. It's best to test these separately. 
The job of the velocity PID is to get the vehicle to a certain velocity as quickly as possible without overshooting. The way to test this is by giving the vehicle a constant command velocity and then observing its response, thus isolating the behavior of the velocity PID. Note that the drag term should correspond to the steady state throttle level necessary to maintain a certain velocity with the vehicle's drag characteristics. Thus, if the vehicle's velocity is oscillating, you might need to tune this drag term. This can actually be done by setting the error term to 0 and seeing if the steady state velocity after the vehicle gets up to speed matches the commanded velocity and adjusting the drag term if not. This likely won't be exact since drag isn't precisely linear with velocity. After finding a good value for the drag term, the gain on the error can be increased to make the vehicle get up to speed faster. If the vehicle starts oscillating around the commanded velocity, then the error gain has been increased too far. 

After tuning the velocity PID using constant command velocities, the command velocity generator can be tuned. The goal of the command velocity generator is to give the velocity PID a velocity that will follow the path. The key is that whenever the vehicle slows down, the command velocity generator must be sure to give achievable velocities to the PID, or the PID will not be able to match the desired velocities and the vehicle will overshoot its target. So, the command velocity generator must be prescient and start slowing down the vehicle ahead of the target. If one wanted to be time-optimal, then the command velocity generator would start slowing down the vehicle at the last possible moment so that the full reverse throttle of the vehicle will cause it decelerate such that it exactly hits the target. However, since there will be some amount of slop in the real world, it's usually better to start slowing down before that and use a gentler deceleration. This is the general line of thinking with this portion of the controller - it's not a big deal to command velocities too high for the vehicle to reach, since the result is just that the vehicle will go slower, but if the vehicle is asked to decelerate too quickly it might overshoot the target or stray off the path.

Keep in mind that the command velocity generator as currently written has no concept of the past - it only knows where it is and where the path is. Thus, it always tries to go the closest point on the path, even if that means skipping earlier parts in the path. This could be potentially changed by adding an array that checks off if the vehicle has gotten within X meters of each point on the path in order, but that is not currently implemented. The advantage of the current scheme is that the path can be resent to the vehicle without worry that the vehicle will start going back to the start of the path. 

How the command velocity generator currently works is that it is a vector sum of a vector towards and a vector parallel to the path. Let's talk about the vector parallel to the path first. 

The vector parallel to the path nominally is just the maximum speed in the direction parallel to the path. At a certain distance from the end of the path, it begins to slow down, becoming linear with the distance. This results in a large initial slowdown and the vehicle's speed getting smaller and smaller as it approaches the target. This actually isn't optimal, as ideally you would maintain the same deceleration over the entire slowdown, allowing you to maintain a higher velocity for longer. But this implementation is easy, and easy to think about - if for example your target velocity is "distance / 2" then the vehicle's velocity will attempt to reach the target point in 2 seconds. The bottom line is that if the vehicle is overshooting the target, increase the amount the distance is divided by. If the vehicle is too slow in getting to the target, decrease the amount the distance is divided by to increase the speed. If the vehicle continues to overshoot the target, you may need to increase the distance at which this slowdown begins. 

The vector parallel is also decreased by a linear factor of the vehicle's distance from the path. This should be pretty obvious - if the vehicle is a certain distance from the path, it will completely eliminate the parallel term and only try to get back on the path using the perpendicular term.

The vector perpendicular to the path is the vector towards the line drawn between the closest point and the point after that point. Note that this means some weirdness may occur if the points are extremely spread out with sharp turns! Be sure to debug to check. In any case, the magnitude of the vector works just like the magnitude of the vector parallel to the path - there is a maximum speed, and if the vehicle is within a certain distance of the path then the speed becomes linear based on distance. If the vehicle is not reaching the path, increase the speed in the linear region (decrease the factor the distance is divided by). If the vehicle is oscillating around the path, decrease the speed in that region. 

The overall velocity magnitude is decreased whenever the vehicle has to turn (whenever the vehicle is pointed away from the desired velocity direction). The velocity is reduced by a linear factor of the angle error up until it is completely zero. If the vehicle is not sufficiently slowing down for turns, likely decrease the speed by increasing the factor the distance is divided by. If the vehicle is slowing down too much, do the opposite. 

Overall, the command velocity generator isn't as mathematically grounded as the velocity PID - it's more of a "feel" thing, since of course it is impossible (and more likely, undesirable) for the vehicle to exactly follow the paths commanded by the navigation system. For example, you want to cut corners to increase speed, or else you'd have to completely stop for every 90 degree turn - most of the time. It is good to work with the navigation engineer to come up with a compromise between speed and accuracy. These can be made without even writing any new software - for example, by sending a series of goals instead of a single path, the vehicle will be more accurate and stop at each goal, which may be desirable. 
