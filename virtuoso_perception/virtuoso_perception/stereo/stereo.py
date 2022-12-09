from sensor_msgs.msg import CameraInfo
from virtuoso_msgs.msg import Contours
import numpy as np
from rclpy.node import Node
import cv2
from ..utils.node_helper import NodeHelper

class Stereo(NodeHelper):

    def __init__(self, node:Node, multiprocessing:bool):

        self._node = node 
        self._multiprocessing = multiprocessing

        self.left_cam_info:CameraInfo = None
        self.right_cam_info:CameraInfo = None

        self.left_buoy_img_contours:Contours = None
        self.right_buoy_img_contours:Contours = None

        # 2 x 3 matrix
        # [ [x translation, y translation, z translation],
        # [rodrigues 1, rodrigues 2, rodrigues 3] ]
        self.cam_transform:np.ndarray = None

        self._left_rect_map:np.ndarray = None
        self._right_rect_map:np.ndarray = None
        self._Q:np.ndarray = None
    
    def _find_intrinsics(cam_info:CameraInfo):
        k = cam_info.k

        camera_matrix = np.array([
            [k[0], 0, k[2]],
            [0, k[4], k[5]],
            [0, 0, 1]
        ]) 
        
        camera_distortion = np.array(cam_info.d)

        return camera_matrix, camera_distortion
    
    def _find_rect_maps(self):
        if not self._left_rect_map is None and not self._right_rect_map is None:
            return

        if self.cam_transform is None:
            return
        
        image_size = (self.left_cam_info.width, self.left_cam_info.height)

        left_matrix, left_distortion = Stereo._find_intrinsics(self.left_cam_info)
        right_matrix, right_distortion = Stereo._find_intrinsics(self.right_cam_info)

        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(left_matrix, left_distortion,
            right_matrix, right_matrix, image_size, self.cam_transform[1], self.cam_transform[0],
            cv2.CALIB_ZERO_DISPARITY)

        self._Q = Q

        self._left_rect_map = cv2.initUndistortRectifyMap(left_matrix, left_distortion, 
            R1, P1, image_size, cv2.CV_32F)
            
        self._right_rect_map = cv2.initUndistortRectifyMap(right_matrix, right_distortion, 
            R2, P2, image_size, cv2.CV_32F)
    
    def run(self):
        self._debug('executing')

        if (self.left_buoy_img_contours is None or self.left_cam_info is None 
            or self.right_buoy_img_contours is None or self.right_cam_info is None):
            self._debug('something is none')
            return

        self._find_rect_maps()
        if self._left_rect_map is None or self._right_rect_map is None:
            return
        self._debug('got rect maps')
        
        if (len(self.left_buoy_img_contours.contour_offsets) != 
            len(self.right_buoy_img_contours.contour_offsets)):
            return
        
        self._debug('got cv2 contours')



