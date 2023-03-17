from .clustering import Clustering
from ..utils.color_range import ColorRange
import numpy as np
import cv2
from cv_bridge import CvBridge
from ..utils.code_identification import find_contours
import random
from scipy import stats

class Cv2ContourFilter(Clustering):

    def __init__(self, node, color_label_bounds:ColorRange, color_filter_bounds:ColorRange,
        buoy_border_px:int, buoy_px_color_sample_size:float):
        super().__init__(node, color_label_bounds, color_filter_bounds)

        self._buoy_border_px = buoy_border_px
        self._buoy_px_color_sample_size = buoy_px_color_sample_size

        self.cv_bridge = CvBridge()

    def __call__(self, bgr_img:np.ndarray):

        img_shape = np.shape(bgr_img)

        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        _, black_and_white = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
        contours = find_contours(black_and_white)

        filtered_contours = list()
        filtered_contour_offsets = list()
        filtered_contour_colors = list()

        self._debug_pub('black_white', 
            self.cv_bridge.cv2_to_imgmsg(black_and_white, encoding='mono8'))
        self._debug_pub('full_contours', 
            self.cv_bridge.cv2_to_imgmsg(cv2.drawContours(
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
