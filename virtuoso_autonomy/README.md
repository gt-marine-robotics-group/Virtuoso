## Virtuoso Autonomy

This package is the "brains" of the robot. It will take in mission data and interpret it, launching and coordinating other parts of the stack accordingly.
For now, the structure of the package is still unclear. However, when working on tasks, simply put the nodes/launch files to be launched in a launch file 
titled appropriatedly (e.g. task3.launch.py). Once we have a plan for structuring the package, we can start integrating mission data to dynamically 
choose what to launch (behavior tree maybe if someone knows C++...).
