from rclpy.node import Node
from typing import List

class ColorRange:

    def __init__(self, node:Node, colors=List[str]):

        self.ranges = {
            'red': {
                'lower1': [0,0,0],
                'upper1': [0,0,0],
                'lower2': [0,0,0],
                'upper2': [0,0,0]
            }, 
            'green': {
                'lower': [0,0,0],
                'upper': [0,0,0]
            },
            'blue': {
                'lower': [0,0,0],
                'upper': [0,0,0]
            },
            'black': {
                'lower': [0,0,0],
                'upper': [0,0,0]
            },
            'yellow': {
                'lower': [0,0,0],
                'upper': [0,0,0]
            },
        }

        self.range_colors = colors

        for color in colors:
            for bound in self.ranges[color]:
                self.ranges[color][bound] = node.get_parameter(
                    f'{color}.{bound}'
                ).value