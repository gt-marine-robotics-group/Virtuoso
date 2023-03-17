import numpy as np
from ..utils.color_range import ColorRange
from ..utils.node_helper import NodeHelper

class Clustering(NodeHelper):

    def __init__(self, node, color_label_bounds:ColorRange, color_filter_bounds:ColorRange):
        super().__init__(node)

        self._color_filter_bounds = color_filter_bounds
        self._color_label_bounds =  color_label_bounds

    def _dominant_color(self, colors:dict):
        dominant = None
        for color, count in colors.items():
            if dominant is None or count > colors[dominant]:
                dominant = color
        
        return dominant
    
    def _pixel_color(self, pixel:np.ndarray):

        for color in self._color_label_bounds.range_colors:
            bounds = list(self._color_label_bounds.ranges[color].values())
            for i in range(0, len(bounds), 2):
                for j in range(len(bounds[i])):
                    if (pixel[j] < bounds[i][j] or pixel[j] > bounds[i + 1][j]):
                        break
                else:
                    return color
        
        return None
    