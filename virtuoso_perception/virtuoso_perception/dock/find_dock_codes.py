from ..utils.color_range import ColorRange
import numpy as np

class FindDockCodes:

    def __init__(self, max_cluster_height:int, min_cluster_height:int,
        max_cluster_width:int, min_cluster_width:int, epsilon:int, min_pts:int,
        code_color_bounds:ColorRange, placard_color_bounds:dict, 
        placard_prop:float):

        self._max_cluster_height = max_cluster_height
        self._min_cluster_height = min_cluster_height
        self._max_cluster_width = max_cluster_width
        self._min_cluster_width = min_cluster_width
        self._epsilon = epsilon
        self._min_pts = min_pts

        self._code_color_bounds = code_color_bounds
        self._placard_color_bounds = placard_color_bounds
        self._placard_prop = placard_prop

        self.node = None
        self.image:np.ndarray = None
    
    def _debug(self, msg:str):
        if self.node is None: return
        self.node.get_logger().info(msg)
    
    def run(self):
        pass