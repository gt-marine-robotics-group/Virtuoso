from enum import Enum

class State(Enum):
    START = 1
    STATION_KEEPING_ENABLED = 2
    WAITING_FOR_SCAN_NODE = 3
    SENDING_SCAN_REQUEST = 4
    SCANNING = 5
    COMPLETE = 6