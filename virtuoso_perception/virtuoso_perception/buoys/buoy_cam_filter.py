import numpy as np
import cv2
from virtuoso_msgs.msg import Contours
from virtuoso_perception.utils.node_helper import NodeHelper
from virtuoso_perception.utils.ColorFilter import ColorFilter
from virtuoso_perception.utils.color_range import ColorRange
from virtuoso_perception.clustering.cv2_contour_filter import Cv2ContourFilter
from virtuoso_perception.clustering.density_filter import DensityFilter

class BuoyFilter(NodeHelper):

    def __init__(self, clustering_method:str, color_filter_bounds:ColorRange, 
        color_label_bounds:ColorRange, buoy_border_px:int, buoy_px_color_sample_size:float, 
        max_cluster_height:int, min_cluster_height:int,
        max_cluster_width:int, min_cluster_width:int, epsilon:int, min_pts:int,
        node=None):

        self._color_filter_bounds = color_filter_bounds

        if clustering_method == 'DENSITY':
            self.clustering = DensityFilter(node, max_cluster_height, min_cluster_height,
                max_cluster_width, min_cluster_width, epsilon, min_pts, buoy_px_color_sample_size,
                color_filter_bounds, color_label_bounds)
        else:
            self.clustering = Cv2ContourFilter(node, color_label_bounds, color_filter_bounds,
                buoy_border_px, buoy_px_color_sample_size)

        self.image:np.ndarray = None

        self.contours:Contours = None
    
    def run(self):

        color_filter = ColorFilter(
            cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV), 
            self.image
        )

        ranges = self._color_filter_bounds.ranges

        red_filtered = color_filter.red_orange_filter(
            hsv_lower1=ranges['red']['lower1'], hsv_upper1=ranges['red']['upper1'], 
            hsv_lower2=ranges['red']['lower2'], hsv_upper2=ranges['red']['upper2'])
        green_filtered = color_filter.green_filter(
            hsv_lower=ranges['green']['lower'], hsv_upper=ranges['green']['upper']
        )
        black_filtered = color_filter.black_filter(
            hsv_lower=ranges['black']['lower'], hsv_upper=ranges['black']['upper']
        )
        yellow_filtered = color_filter.yellow_filter(
            hsv_lower=ranges['yellow']['lower'], hsv_upper=ranges['yellow']['upper']
        )

        combo = cv2.bitwise_or(
            cv2.bitwise_or(cv2.bitwise_or(red_filtered, green_filtered), yellow_filtered),
            black_filtered 
        )

        contours, colors, contour_offsets = self.clustering(combo)

        msg = Contours()
        msg.contour_points = contours
        msg.contour_offsets = contour_offsets
        msg.contour_colors = colors

        self.contours = msg
    