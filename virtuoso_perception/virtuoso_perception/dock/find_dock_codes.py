from ..utils.color_range import ColorRange
from ..utils.ColorFilter import ColorFilter
from ..clustering.density_filter import DensityFilter
from ..utils.node_helper import NodeHelper
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FindDockCodes(NodeHelper):

    def __init__(self, max_cluster_height:int, min_cluster_height:int,
        max_cluster_width:int, min_cluster_width:int, epsilon:int, min_pts:int,
        code_px_color_sample_size:float,
        code_color_bounds:ColorRange, placard_color_bounds:dict, 
        placard_prop:float, node):
        super().__init__(node)

        self._code_color_bounds = code_color_bounds

        self._clustering = DensityFilter(node, max_cluster_height, min_cluster_height,
            max_cluster_width, min_cluster_width, epsilon, min_pts, code_px_color_sample_size,
            code_color_bounds, code_color_bounds)

        self._placard_color_bounds = placard_color_bounds
        self._placard_prop = placard_prop

        self.image:Image = None

        self._cv_bridge = CvBridge()
    
    def run(self):

        bgr_image = self._cv_bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')

        color_filter = ColorFilter(
            cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV),
            bgr_image
        )

        ranges = self._code_color_bounds.ranges

        red_filtered = color_filter.red_orange_filter(
            hsv_lower1=ranges['red']['lower1'], hsv_upper1=ranges['red']['upper1'], 
            hsv_lower2=ranges['red']['lower2'], hsv_upper2=ranges['red']['upper2'])
        green_filtered = color_filter.green_filter(
            hsv_lower=ranges['green']['lower'], hsv_upper=ranges['green']['upper']
        )
        blue_filtered = color_filter.blue_filter(
            hsv_lower=ranges['blue']['lower'], hsv_upper=ranges['blue']['upper']
        )

        combo = cv2.bitwise_or(cv2.bitwise_or(red_filtered, green_filtered), blue_filtered)

        contours, colors, contour_offsets = self._clustering(combo, contour_color=(193,182,255))

        placard_filter = color_filter.filter(lower=np.array(self._placard_color_bounds['lower']),
            upper=np.array(self._placard_color_bounds['upper']))
        
        self._debug_pub('placard_bg_filter', 
            self._cv_bridge.cv2_to_imgmsg(placard_filter, encoding='bgr8')) 
