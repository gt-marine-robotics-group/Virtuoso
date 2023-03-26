from enum import Enum

class State(Enum):
    START = 1
    TASK_WAYPOINT_NAVIGATING = 2
    DOCKING_STOP = 3
    BALL_SHOOTING = 4
    COMPLETE = 5
