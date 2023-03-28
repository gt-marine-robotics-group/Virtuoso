from enum import Enum

class State(Enum):
    START = 1
    TASK_WAYPOINT_NAVIGATING = 2
    T1_FINDING_NEXT_GATE = 3
    T1_GATE_NAVIGATING = 4
    T1_EXTRA_FORWARD_NAVIGATING = 5
    T1_FINAL_EXTRA_FORWARD_NAVIGATING = 6
    COMPLETE = 7