from enum import Enum

class State(Enum):
    SEARCHING = 1
    SEARCH_NAVIGATING = 2
    ANIMAL_NAVIGATING = 3
    COMPLETE = 4