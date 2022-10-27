from math import sqrt
from typing import Tuple

def distance(a:Tuple, b:Tuple):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)