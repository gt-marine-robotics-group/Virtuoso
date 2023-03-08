from rclpy.node import Node
import numpy as np
import cv2
from virtuoso_msgs.msg import Contours
from ..utils.ColorFilter import ColorFilter
from ..utils.color_range import ColorRange
from ..utils.code_identification import find_contours
from cv_bridge import CvBridge
import random
from scipy import stats
from collections import deque

class BuoyFilter:

    def __init__(self, clustering_method:str, color_filter_bounds:ColorRange, 
        color_label_bounds:ColorRange, buoy_border_px:int, buoy_px_color_sample_size:float, 
        max_cluster_px:int, min_cluster_px:int, epsilon:int, min_pts:int):
        
        self._color_filter_bounds = color_filter_bounds
        self._color_label_bounds =  color_label_bounds

        self._buoy_border_px = buoy_border_px
        self._buoy_px_color_sample_size = buoy_px_color_sample_size

        self._max_cluster_px = max_cluster_px
        self._min_cluster_px = min_cluster_px
        self._epsilon = epsilon
        self._min_pts = min_pts

        self._clustering_method = clustering_method

        self.node:Node = None
        self.image:np.ndarray = None

        self.contours:Contours = None
    
    def _debug_pub(self, name:str, msg):
        if self.node is None:
            return
        self.node.debug_pubs[name].publish(msg)
    
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

        if self._clustering_method == 'DENSITY':
            contours, colors, contour_offsets = self._density_filter(combo)
        else:
            contours, colors, contour_offsets = self._cv2_contour_filter(combo)

        msg = Contours()
        msg.contour_points = contours
        msg.contour_offsets = contour_offsets
        msg.contour_colors = colors

        self.contours = msg
    
    def _density_filter(self, bgr_img:np.ndarray):
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        _, black_and_white = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)

        hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

        self._debug_pub('black_white', 
            CvBridge().cv2_to_imgmsg(black_and_white, encoding='mono8'))

        colored = np.where(black_and_white == 255)

        clusters = list()
        cluster_bounds = list()
        visited = set()

        curr_cluster_queue = deque()

        for i in range(colored[0].shape[0]):
            if i in visited:
                continue
        
            if (len(clusters) > 0 and 
                (len(clusters[-1]) < self._min_cluster_px or len(clusters[-1]) > self._max_cluster_px)):
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
                    # self.node.get_logger().info(f'visited size: {len(visited)}')
                    curr_cluster_queue.append(c)

        if (len(clusters) > 0 and 
            (len(clusters[-1]) < self._min_cluster_px or len(clusters[-1]) > self._max_cluster_px)):
            clusters.pop()
        
        contours = list()

        for c in range(len(clusters)):
            bounds = cluster_bounds[c]
            contour = self._create_contour_from_bounds(bounds, colored)
            contours.append(contour)

        self._debug_pub('full_contours', 
            CvBridge().cv2_to_imgmsg(cv2.drawContours(
                    bgr_img.copy(), tuple(contours), -1, (255,0,0), 1
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
                'black': 0 
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
    
    def _create_contour_from_bounds(self, bounds, colored):
        y_points = np.arange(start=colored[0][bounds['top']], stop=colored[0][bounds['bottom']] + 1, dtype=int)
        x_points = np.arange(start=colored[1][bounds['left']], stop=colored[1][bounds['right']] + 1, dtype=int)

        contour = np.zeros((2 * y_points.shape[0] + 2 * x_points.shape[0] - 3, 2), dtype=int)

        contour[:y_points.shape[0],1] = y_points
        contour[:y_points.shape[0],0] = x_points[0]

        contour[y_points.shape[0]:y_points.shape[0] + x_points.shape[0] - 1,1] = y_points[-1]
        contour[y_points.shape[0]:y_points.shape[0] + x_points.shape[0] - 1,0] = np.arange(x_points[0] + 1, x_points[-1] + 1)

        contour[y_points.shape[0] + x_points.shape[0] - 1:2 * y_points.shape[0] + x_points.shape[0] - 2,1] = np.arange(y_points[-1] - 1, y_points[0] - 1, -1)
        contour[y_points.shape[0] + x_points.shape[0] - 1:2 * y_points.shape[0] + x_points.shape[0] - 2,0] = x_points[-1]

        contour[2 * y_points.shape[0] + x_points.shape[0] - 2:,1] = y_points[0]
        contour[2 * y_points.shape[0] + x_points.shape[0] - 2:,0] = np.arange(x_points[-1] - 1, x_points[0] - 1, -1)

        return contour[:,np.newaxis,:]
    
    def _cv2_contour_filter(self, bgr_img:np.ndarray):

        img_shape = np.shape(bgr_img)

        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        _, black_and_white = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
        contours = find_contours(black_and_white)

        filtered_contours = list()
        filtered_contour_offsets = list()
        filtered_contour_colors = list()

        self._debug_pub('black_white', 
            CvBridge().cv2_to_imgmsg(black_and_white, encoding='mono8'))
        self._debug_pub('full_contours', 
            CvBridge().cv2_to_imgmsg(cv2.drawContours(
                    bgr_img.copy(), contours, -1, (255,0,0), 1
                ),
                encoding='bgr8'
            )
        )

        hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)

        filtered = list()
        for i, cnt in enumerate(contours):

            area = cv2.contourArea(cnt)
            if area < 100:
                continue

            # create image with specific contour filled in
            blank = np.zeros((img_shape[0], img_shape[1]))
            filled = cv2.drawContours(blank, contours, i, 255, -1)
            
            # get the index of all pixels within the contour
            pts = np.where(filled == 255)

            x_to_y = dict()
            y_to_x = dict()

            for pt in cnt:
                pt = pt[0]
                if pt[0] in x_to_y:
                    x_to_y[pt[0]].append(pt[1])
                else:
                    x_to_y[pt[0]] = [pt[1]]
                
                if pt[1] in y_to_x:
                    y_to_x[pt[1]].append(pt[0])
                else:
                    y_to_x[pt[1]] = [pt[0]]

            # determine whether pixel is or isn't filtered out
            binary_pixels = list()
            colors = {
                'red': 0,
                'green': 0,
                'yellow': 0,
                'black': 0 
            }
            for i in range(pts[0].size):
                on_border = False

                if pts[0][i] in x_to_y:
                    for pt in x_to_y[pts[0][i]]:
                        if abs(pt - pts[1][i]) < self._buoy_border_px:
                            on_border = True
                            break
                if on_border: continue

                if pts[1][i] in y_to_x:
                    for pt in y_to_x[pts[1][i]]:
                        if abs(pt - pts[0][i]) < self._buoy_border_px:
                            on_border = True
                            break
                if on_border: continue

                if random.randint(0, 99) < self._buoy_px_color_sample_size * 100: 
                    color = self._pixel_color(hsv_img[pts[0][i]][pts[1][i]])
                    if not color is None:
                        colors[color] += 1

                binary_pixels.append(black_and_white[pts[0][i]][pts[1][i]])

            mode = stats.mode(binary_pixels)

            if mode.mode[0] != 255:
                continue
            
            if mode.count[0] / len(binary_pixels) < .70:
                continue

            filtered_contour_offsets.append(len(filtered_contours))
            filtered_contours.extend(int(p) for pt in cnt for p in pt[0])
            filtered_contour_colors.append(self._dominant_color(colors))
            filtered.append(cnt)

        drawn = cv2.drawContours(bgr_img.copy(), filtered, -1, (255,0,0), 3)
        self._debug_pub('filtered_contours', 
            CvBridge().cv2_to_imgmsg(drawn, encoding='bgr8')
        )

        return filtered_contours, filtered_contour_colors, filtered_contour_offsets
    
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
    