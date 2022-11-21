from enum import Enum

class State(Enum):
    START = 1
    STATION_KEEPING_ENABLED = 2
    SENDING_SCAN_REQUEST = 3
    SCANNING = 4
    COMPLETE = 5