from .clustering import Clustering
from ..utils.color_range import ColorRange
import cv2
import numpy as np
from cv_bridge import CvBridge
from collections import deque
import random

class DensityFilter(Clustering):

    def __init__(self, node, max_cluster_height:int, min_cluster_height:int,
        max_cluster_width:int, min_cluster_width:int, epsilon:int, min_pts:int,
        buoy_px_color_sample_size:float,
        color_filter_bounds:ColorRange, color_label_bounds:ColorRange):
        super().__init__(node, color_label_bounds, color_filter_bounds)

        self._buoy_px_color_sample_size = buoy_px_color_sample_size
        self._max_cluster_height = max_cluster_height
        self._min_cluster_height = min_cluster_height
        self._max_cluster_width = max_cluster_width
        self._min_cluster_width = min_cluster_width
        self._epsilon = epsilon
        self._min_pts = min_pts

        self.cv_bridge = CvBridge()

    def __call__(self, bgr_img:np.ndarray, contour_color=(255,0,0)):
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        _, black_and_white = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)

        hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

        self._debug_pub('black_white', 
            self.cv_bridge.cv2_to_imgmsg(black_and_white, encoding='mono8'))

        colored = np.where(black_and_white == 255)

        clusters = list()
        cluster_bounds = list()
        visited = set()

        curr_cluster_queue = deque()

        for i in range(colored[0].shape[0]):
            if i in visited:
                continue
        
            if (len(clusters) > 0 and not self._cluster_meets_bounding_reqs(colored, cluster_bounds[-1])):
                clusters[-1] = set()
                cluster_bounds[-1] = {'left': i, 'right': i, 'top': i, 'bottom': i}
            else:
                clusters.append(set())                
                cluster_bounds.append({'left': i, 'right': i, 'top': i, 'bottom': i})

            visited.add(i)

            curr_cluster_queue.append(i)

            while len(curr_cluster_queue) > 0:
                index = curr_cluster_queue.pop()
                clusters[-1].add(index)

                y = colored[0][index]
                x = colored[1][index]

                if y < colored[0][cluster_bounds[-1]['top']]:
                    cluster_bounds[-1]['top'] = index
                elif y > colored[0][cluster_bounds[-1]['bottom']]:
                    cluster_bounds[-1]['bottom'] = index
                if x < colored[1][cluster_bounds[-1]['left']]:
                    cluster_bounds[-1]['left'] = index
                elif x > colored[1][cluster_bounds[-1]['right']]:
                    cluster_bounds[-1]['right'] = index

                neighbors = np.where(
                    np.square(colored[0] - y) + np.square(colored[1] - x) <= self._epsilon**2
                )

                if neighbors[0].shape[0] < self._min_pts:
                    continue

                for c_i in range(neighbors[0].shape[0]):
                    c = neighbors[0][c_i]
                    if c in visited: continue
                    visited.add(c)
                    curr_cluster_queue.append(c)

        if (len(clusters) > 0 and not self._cluster_meets_bounding_reqs(colored, cluster_bounds[-1])):
            clusters.pop()
        
        contours = list()

        for c in range(len(clusters)):
            bounds = cluster_bounds[c]
            contour = self._create_contour_from_bounds(bounds, colored)
            contours.append(contour)

        self._debug_pub('full_contours', 
            self.cv_bridge.cv2_to_imgmsg(cv2.drawContours(
                    bgr_img.copy(), tuple(contours), -1, contour_color, 1
                ),
                encoding='bgr8'
            )
        )

        filtered_contours = list()
        filtered_contour_offsets = list()
        filtered_contour_colors = list()

        for i, cnt in enumerate(contours):
            filtered_contour_offsets.append(len(filtered_contours))
            filtered_contours.extend(int(c) for c in cnt.flatten())

            colors = {
                'red': 0,
                'green': 0,
                'yellow': 0,
                'black': 0,
                'blue': 0,
                'white': 0
            }

            blank = np.zeros((bgr_img.shape[0], bgr_img.shape[1]))
            filled = cv2.drawContours(blank, contours, i, 255, -1)
            
            pts = np.where(filled == 255)

            for i in range(pts[0].size):
                rand_int = random.randint(0, 99) 
                if rand_int < self._buoy_px_color_sample_size * 100: 
                    color = self._pixel_color(hsv_img[pts[0][i]][pts[1][i]])
                    if not color is None:
                        colors[color] += 1
            
            filtered_contour_colors.append(self._dominant_color(colors))
        
        return filtered_contours, filtered_contour_colors, filtered_contour_offsets
    
    def _cluster_meets_bounding_reqs(self, colored, bounds):
        y_diff = colored[0][bounds['bottom']] - colored[0][bounds['top']]
        x_diff = colored[1][bounds['right']] - colored[1][bounds['left']]

        if y_diff > self._max_cluster_height: return False
        if y_diff < self._min_cluster_height: return False
        if x_diff > self._max_cluster_width: return False
        if x_diff < self._min_cluster_width: return False

        return True
    
    def _create_contour_from_bounds(self, bounds, colored):
        y_points = np.arange(start=colored[0][bounds['top']], stop=colored[0][bounds['bottom']] + 1, dtype=int)
        x_points = np.arange(start=colored[1][bounds['left']], stop=colored[1][bounds['right']] + 1, dtype=int)

        contour = np.zeros((2 * y_points.shape[0] + 2 * x_points.shape[0] - 4, 2), dtype=int)

        contour[:y_points.shape[0],1] = y_points
        contour[:y_points.shape[0],0] = x_points[0]

        contour[y_points.shape[0]:y_points.shape[0] + x_points.shape[0] - 1,1] = y_points[-1]
        contour[y_points.shape[0]:y_points.shape[0] + x_points.shape[0] - 1,0] = np.arange(x_points[0] + 1, x_points[-1] + 1)

        contour[y_points.shape[0] + x_points.shape[0] - 1:2 * y_points.shape[0] + x_points.shape[0] - 2,1] = np.arange(y_points[-1] - 1, y_points[0] - 1, -1)
        contour[y_points.shape[0] + x_points.shape[0] - 1:2 * y_points.shape[0] + x_points.shape[0] - 2,0] = x_points[-1]

        contour[2 * y_points.shape[0] + x_points.shape[0] - 2:,1] = y_points[0]
        contour[2 * y_points.shape[0] + x_points.shape[0] - 2:,0] = np.arange(x_points[-1] - 1, x_points[0], -1)

        return contour[:,np.newaxis,:]