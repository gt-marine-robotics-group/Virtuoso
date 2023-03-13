from sensor_msgs.msg import CameraInfo, PointCloud2
from virtuoso_msgs.msg import Contours, BuoyArray
import numpy as np
from rclpy.node import Node
import cv2
from ..utils.node_helper import NodeHelper
from virtuoso_perception.utils.pointcloud import create_cloud_xyz32
from cv_bridge import CvBridge

class Stereo(NodeHelper):

    def __init__(self, node:Node, multiprocessing:bool):

        super().__init__(node)
        self._multiprocessing = multiprocessing

        self._cv_bridge = CvBridge()

        self.left_cam_info:CameraInfo = None
        self.right_cam_info:CameraInfo = None

        # 2 x 3 matrix
        # [ [x translation, y translation, z translation],
        # [rodrigues 1, rodrigues 2, rodrigues 3] ]
        self.cam_transform:np.ndarray = None

        self._left_rect_map:np.ndarray = None
        self._right_rect_map:np.ndarray = None
        self._Q:np.ndarray = None
    
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
