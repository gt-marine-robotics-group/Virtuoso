from .stereo import Stereo
from virtuoso_msgs.msg import Contours
from geometry_msgs.msg import Point
from typing import List

class DockStereo(Stereo):

    def __init__(self, node, multiprocessing:bool):
        super().__init__(node, multiprocessing)

        self.left_img_contours:Contours = None
        self.right_img_contours:Contours = None

        self.end_points:List[Point] = None
    
    def run(self):
        self._debug('executing')
