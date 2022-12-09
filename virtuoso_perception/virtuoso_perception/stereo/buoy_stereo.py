from rclpy.node import Node
from ..utils.node_helper import NodeHelper
import numpy as np
import cv2
from virtuoso_msgs.msg import Contours
from .utils import unflatten_contours
from multiprocessing import Process, Array
from sensor_msgs.msg import CameraInfo

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

    def _update_debug_pub_sizes(self, num:int):
        if self._node is None:
            return
        self._node.update_debug_pub_sizes(num)
    
    def run(self):

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
            return
        
        self._update_debug_pub_sizes(len(buoy_pairs))

        self._debug_pub(f'pub length: {len(buoy_pairs)}')

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
            
            for p in processes:
                p.join()
        
    def _find_buoy_pose(self, left_img, right_img, buoy_index):
        # run_stereo from stereo_node.py
        pass