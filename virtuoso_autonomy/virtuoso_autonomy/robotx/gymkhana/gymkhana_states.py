from enum import Enum

class State(Enum):
    START = 1
    STATION_KEEPING_ENABLED = 2
    FINDING_NEXT_GATE = 3
    NAVIGATING = 4
    COMPLETE = 5