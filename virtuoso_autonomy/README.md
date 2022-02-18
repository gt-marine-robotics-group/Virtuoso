# Virtuoso Autonomy

This package is the "brains" of the robot. It will take in mission data and interpret it, coordinating other parts of the stack accordingly.

Each [task](./virtuoso_autonomy/tasks) has a main node that is launched by the [main launch file](./launch/main.launch.py). These nodes handle the interpretation of mission data.

**NOTE:** For the mission_interpreter to work, there must be a custom bridge for the Task message. Documentation on building this bridge with custom messages can be found [here](https://github.com/gt-marine-robotics-group/Virtuoso-Messages/tree/main).

## Tasks

### Task 3: Perception
The main Perception node subscribes to the task info, classified buoys, and filtered gps topics. Using this information, it sends one message per buoy identified. It only sends a message identifying a buoy after reviewing multiple messages from the classified buoys.
