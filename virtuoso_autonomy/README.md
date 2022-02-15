## Virtuoso Autonomy

This package is the "brains" of the robot. It will take in mission data and interpret it, launching and coordinating other parts of the stack accordingly.

The mission_interpreter node takes in the Task message sent by VRX and depending on the name of the task, runs the appropriate callback function.

**NOTE:** For the mission_interpreter to work, there must be a custom bridge for the Task message. Documentation on building this bridge is still in progress. If you message me (Manuel) I can help walk you through it.
