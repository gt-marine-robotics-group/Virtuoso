from enum import Enum

class State(Enum):
    START = 1
    STATION_KEEPING_ENABLED = 2
    FINDING_GATE = 3
    NAVIGATING_TO_GATE_MIDPOINT = 4
    EXTRA_FORWARD_NAV = 5
    CHECKING_FOR_LOOP_BUOY = 6
    NAVIGATING_STRAIGHT = 7
    LOOPING = 8
    COMPLETE = 9