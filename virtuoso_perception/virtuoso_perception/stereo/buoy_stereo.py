from rclpy.node import Node
from ..utils.node_helper import NodeHelper
import numpy as np
import cv2
from virtuoso_msgs.msg import Contours, Buoy, BuoyArray
from geometry_msgs.msg import Point
from .utils import unflatten_contours, img_points_to_physical_xy
from multiprocessing import Process, Array
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from .pixel_matcher import PixelMatcher
import time

class BuoyStereo(NodeHelper):

    def __init__(self, node:Node, multiprocessing:bool, 
        left_contours:Contours, right_contours:Contours,
        left_rect_map:np.ndarray, right_rect_map:np.ndarray,
        left_cam_info:CameraInfo, right_cam_info:CameraInfo):

        self._multiprocessing = multiprocessing
        self._node = node

        self._left_contours = left_contours
        self._right_contours = right_contours

        self._left_rect_map = left_rect_map
        self._right_rect_map = right_rect_map

        self._left_cam_info = left_cam_info
        self._right_cam_info = right_cam_info

        self._points = list()

        self._cv_bridge = CvBridge()

    def _update_debug_pub_sizes(self, num:int):
        if self._node is None:
            return
        self._node.update_debug_pub_sizes(num)
    
    def find_buoys(self):

        contours = [
            unflatten_contours(self._left_contours.contour_points, 
                self._left_contours.contour_offsets),
            unflatten_contours(self._right_contours.contour_points, 
                self._right_contours.contour_offsets)
        ]

        buoy_pairs = list()
        for cnt_num in range(len(self._left_contours.contour_offsets)):
            pair = list()
            for img_num in range(2):
                blank = np.zeros((self._left_cam_info.height, self._left_cam_info.width))
                filled = cv2.drawContours(blank, contours[img_num], cnt_num, 255, 1).astype('uint8')
                pair.append(filled)
            buoy_pairs.append(pair)

        if len(buoy_pairs) == 0:
            self._debug('no contours')
            return BuoyArray()
        
        self._update_debug_pub_sizes(len(buoy_pairs))

        self._debug(f'pub length: {len(buoy_pairs)}')

        self._points = Array(typecode_or_type='d', size_or_initializer=len(buoy_pairs)*2)

        contours_remaining = True
        count = 0
        while contours_remaining:
            contours_remaining = False
            # run at most 8 processes
            pair_indexes = ((count * 8) + i for i in range(8))
            processes = list()
            for index in pair_indexes:
                if index >= len(buoy_pairs): break
                process = Process(target=self._find_buoy_pose, 
                    args=(buoy_pairs[index][0], buoy_pairs[index][1], index))
                process.start()
                processes.append(process)
            else:
                contours_remaining = True
                count += 1
            
            start = time.time()
            while time.time() - start <= 5:
                self._debug('waiting for processes')
                if any(p.is_alive() for p in processes):
                    break
                time.sleep(1.0)
            else:
                self._debug('process took to long')
                for p in processes:
                    p.terminate()
                    p.join()
                return BuoyArray()
            
            for p in processes:
                p.join()
            
            for p in processes:
                p.close()
        
        buoys = BuoyArray()
        for i in range(len(self._points) // 2):
            buoy = Buoy(location=Point(x=self._points[i * 2], y=self._points[(i * 2) + 1]))
            buoys.buoys.append(buoy)
        
        return buoys
        
    def _find_buoy_pose(self, left_img, right_img, buoy_index):
        self._debug(f'buoy_index: {buoy_index}')

        self._debug_pub_indexed('/perception/stereo/debug/left_cam/contoured_buoy', buoy_index,
            self._cv_bridge.cv2_to_imgmsg(left_img, encoding='mono8')) 
        self._debug_pub_indexed('/perception/stereo/debug/right_cam/contoured_buoy', buoy_index,
            self._cv_bridge.cv2_to_imgmsg(right_img, encoding='mono8'))

        left_img_rect = cv2.remap(left_img, *self._left_rect_map, cv2.INTER_LANCZOS4)
        right_img_rect = cv2.remap(right_img, *self._right_rect_map, cv2.INTER_LANCZOS4)

        self._debug_pub_indexed('/perception/stereo/debug/left_cam/rectified_buoy', buoy_index,
            self._cv_bridge.cv2_to_imgmsg(left_img_rect, encoding='mono8'))
        self._debug_pub_indexed('/perception/stereo/debug/right_cam/rectified_buoy', buoy_index,
            self._cv_bridge.cv2_to_imgmsg(right_img_rect, encoding='mono8'))

        midpoints = PixelMatcher.midpoints(left_img_rect, right_img_rect)

        self._debug(f'midpoints: {midpoints}')

        x, y = img_points_to_physical_xy(midpoints[0], midpoints[1], 
            self._left_cam_info.k[0], self._right_cam_info.k[0], 
            (len(left_img_rect) // 2, len(left_img_rect[0]) // 2)
        )

        points_base_index = buoy_index * 2
        self._points[points_base_index] = x
        self._points[points_base_index + 1] = y

