from enum import Enum

class State(Enum):
    SEARCHING = 1
    SEARCH_TRANSLATING = 2
    SEARCH_TRANSLATING_DONE = 3
    SEARCH_ROTATING1 = 4
    SEARCH_ROTATING1_DONE = 5
    SEARCH_ROTATING2 = 6
    SEARCH_ROTATING2_DONE = 7
    ANIMAL_NAVIGATING = 8
    COMPLETE = 9