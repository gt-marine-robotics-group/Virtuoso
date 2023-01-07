from enum import Enum

class State(Enum):
    START = 1
    STATION_KEEPING_ENABLED = 2
    FINDING_NEXT_GATE = 3
    ROTATING_TO_FIND_NEXT_GATE = 4
    NAVIGATING = 5
    COMPLETE = 6