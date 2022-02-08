# Virtuoso Navigation

## Nodes

### /set_goal
The SetGoal node takes in a desired goal (published to `/virtuoso_navigation/set_goal`). 
The goal is passed on to nav2 which generates a [Path](https://github.com/ros2/common_interfaces/blob/master/nav_msgs/msg/Path.msg) published to `/plan`.
Nav2 also publishes a velocity to `/cmd_vel` that will be used by our motors.
