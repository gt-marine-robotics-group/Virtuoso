from sensor_msgs.msg import CameraInfo, PointCloud2
from virtuoso_msgs.msg import Contours, BuoyArray
import numpy as np
from rclpy.node import Node
import cv2
from ..utils.node_helper import NodeHelper
from virtuoso_processing.utils.pointcloud import create_cloud_xyz32
from .buoy_stereo import BuoyStereo

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

        self.buoys:BuoyArray = None
    
    def _debug_pcd(self, buoys:BuoyArray):
        pcd = PointCloud2()
        pcd.header.frame_id = 'wamv/lidar_wamv_link'
        pcd_points = list()
        for buoy in buoys.buoys:
            pcd_points.append([buoy.location.x, buoy.location.y, 0])
        
        pcd = create_cloud_xyz32(pcd.header, pcd_points)

        self._debug_pub('/perception/stereo/debug/points', pcd)
    
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
            right_matrix, right_distortion, image_size, self.cam_transform[1], self.cam_transform[0],
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

        buoy_stereo = BuoyStereo(node=self._node, multiprocessing=self._multiprocessing, 
            left_contours=self.left_buoy_img_contours, 
            right_contours=self.right_buoy_img_contours, left_rect_map=self._left_rect_map,
            right_rect_map=self._right_rect_map, left_cam_info=self.left_cam_info,
            right_cam_info=self.right_cam_info)
        
        self.buoys = buoy_stereo.find_buoys()

        self._debug_pcd(self.buoys)

