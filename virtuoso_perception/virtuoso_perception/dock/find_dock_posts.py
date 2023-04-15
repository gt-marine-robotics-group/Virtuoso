from virtuoso_perception.utils.color_range import ColorRange
from virtuoso_perception.utils.ColorFilter import ColorFilter
from virtuoso_perception.clustering.density_filter import DensityFilter
from virtuoso_perception.utils.node_helper import NodeHelper
from virtuoso_perception.stereo.utils import unflatten_contours
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FindDockPosts(NodeHelper):

    def __init__(self, max_cluster_height:int, min_cluster_height:int,
        max_cluster_width:int, min_cluster_width:int, epsilon:int, min_pts:int,
        post_px_color_sample_size:float, post_px_density:float,
        post_color_bounds:ColorRange, placard_color_bounds:dict, 
        placard_prop:float, placard_search_range:int, node):
        super().__init__(node)

        self._post_color_bounds = post_color_bounds

        self._clustering = DensityFilter(node, max_cluster_height, min_cluster_height,
            max_cluster_width, min_cluster_width, epsilon, min_pts, post_px_color_sample_size,
            post_color_bounds, post_color_bounds)
        
        self._post_px_density = post_px_density

        self._placard_search_range = placard_search_range
        self._placard_color_bounds = placard_color_bounds
        self._placard_prop = placard_prop

        self.image:Image = None

        self._cv_bridge = CvBridge()
    
    def run(self):

        bgr_image = self._cv_bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        color_filter = ColorFilter(
            hsv_image,
            bgr_image
        )

        ranges = self._post_color_bounds.ranges

        red_filtered = color_filter.red_orange_filter(
            hsv_lower1=ranges['red']['lower1'], hsv_upper1=ranges['red']['upper1'], 
            hsv_lower2=ranges['red']['lower2'], hsv_upper2=ranges['red']['upper2'])
        green_filtered = color_filter.green_filter(
            hsv_lower=ranges['green']['lower'], hsv_upper=ranges['green']['upper']
        )
        white_filtered = color_filter.white_filter(
            hsv_lower=ranges['white']['lower'], hsv_upper=ranges['white']['upper']
        )

        combo = cv2.bitwise_or(cv2.bitwise_or(red_filtered, green_filtered), white_filtered)

        contours, colors, contour_offsets = self._clustering(combo, contour_color=(193,182,255))

        placard_filter = color_filter.filter(lower=np.array(self._placard_color_bounds['lower']),
            upper=np.array(self._placard_color_bounds['upper']))

        self._debug_pub('placard_bg_filter', 
            self._cv_bridge.cv2_to_imgmsg(placard_filter, encoding='bgr8')) 

        contours = unflatten_contours(contours, contour_offsets)

        contours, bounds, colors = self._filter_contours_by_placard_backdrop(contours, colors, hsv_image)

        contours, colors = self._filter_contours_by_px_density(contours, bounds, colors, hsv_image)

        self._debug_pub('posts', self._cv_bridge.cv2_to_imgmsg(cv2.drawContours(
            combo.copy(), tuple(contours), -1, (193,182,255), 1
        ), encoding='bgr8')) 

        flattened_contours = list()
        contour_offsets = list()

        for contour in contours:
            contour_offsets.append(len(flattened_contours))
            for pt in contour:
                flattened_contours.extend([int(pt[0][0]), int(pt[0][1])])    
        
        return flattened_contours, contour_offsets, colors
    
    def _filter_contours_by_px_density(self, contours, bounds, colors, hsv):
        filtered_contours = list()
        filtered_colors = list()

        for contour_index in range(len(contours)):
            bound = bounds[contour_index]
            color = colors[contour_index]

            x_diff = bound['right'] - bound['left']
            left = int(bound['left'] + (.25 * x_diff))
            right = int(bound['right'] - (.25 * x_diff) + 1)

            pxs = hsv[bound['top']:bound['bottom'] + 1,left:right]

            if color == 'red':
                mask = cv2.inRange(pxs, np.array(self._post_color_bounds.ranges[color]['lower1']),
                    np.array(self._post_color_bounds.ranges[color]['upper1']))
                mask = cv2.bitwise_or(mask, cv2.inRange(pxs, np.array(self._post_color_bounds.ranges[color]['lower2']),
                    np.array(self._post_color_bounds.ranges[color]['upper2'])))
            else:
                mask = cv2.inRange(pxs, np.array(self._post_color_bounds.ranges[color]['lower']),
                    np.array(self._post_color_bounds.ranges[color]['upper']))
            
            num_nonzero = np.count_nonzero(mask)

            prop = num_nonzero / (pxs.shape[0] * pxs.shape[1])

            self._debug(f'prop: {prop}')

            if prop > self._post_px_density:
                filtered_contours.append(contours[contour_index])
                filtered_colors.append(color)

        return filtered_contours, filtered_colors


    def _filter_contours_by_placard_backdrop(self, contours, colors, hsv):

        search_range = self._placard_search_range

        filtered_contours = list()
        filtered_bounds = list()
        filtered_colors = list()

        for contour_index in range(contours.shape[0]):
            contour = contours[contour_index]

            bounds = self._find_contour_bounds(contour[:,0,:])

            if bounds['top'] - search_range >= 0:
                top = hsv[bounds['top'] - search_range : bounds['top'], bounds['left']:bounds['right'] + 1,:]
            else: top = np.ndarray((0,0,3), dtype=int)

            if bounds['bottom'] + search_range <= hsv.shape[0]:
                bottom = hsv[bounds['bottom'] + 1 : bounds['bottom'] + search_range + 1, bounds['left']:bounds['right'] + 1,:]
            else: bottom = np.ndarray((0,0,3), dtype=int)

            if bounds['left'] - search_range >= 0:
                left = hsv[bounds['top']:bounds['bottom'] + 1, bounds['left'] - search_range : bounds['left'],:]
            else: left = np.ndarray((0,0,3), dtype=int)

            if bounds['right'] + search_range <= hsv.shape[1]:
                right = hsv[bounds['top']:bounds['bottom'] + 1, bounds['right'] + 1 : bounds['right'] + search_range + 1,:]
            else: right = np.ndarray((0,0,3), dtype=int)

            total = np.append(np.reshape(top, (-1,3)), np.reshape(bottom, (-1,3)), axis=0)
            total = np.append(total, np.reshape(left, (-1,3)), axis=0)
            total = np.append(total, np.reshape(right, (-1,3)), axis=0)

            mask = cv2.inRange(total[np.newaxis,:,:], np.array(self._placard_color_bounds['lower']),
                np.array(self._placard_color_bounds['upper']))

            num_nonzero = np.count_nonzero(mask)

            prop = num_nonzero / total.shape[0]

            if prop < self._placard_prop:
                filtered_contours.append(contour)
                filtered_bounds.append(bounds)
                filtered_colors.append(colors[contour_index])
        
        return filtered_contours, filtered_bounds, filtered_colors
            

    def _find_contour_bounds(self, contour):

        bounds = {
            'bottom': np.max(contour[:,1]),
            'top': np.min(contour[:,1]),
            'left': np.min(contour[:,0]),
            'right': np.max(contour[:,0])
        }

        return bounds