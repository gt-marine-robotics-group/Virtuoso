from rclpy.node import Node
from .stereo import Stereo
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2
from virtuoso_msgs.msg import Contours, Buoy, BuoyArray
from geometry_msgs.msg import Point
from .utils import unflatten_contours, img_points_to_physical_xy, contour_average_yx
from ..utils.pointcloud import create_cloud_xyz32
from multiprocessing import Process, Array
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from .pixel_matcher import PixelMatcher
import time

class BuoyStereo(Stereo):

    def __init__(self, node:Node, multiprocessing:bool):
        super().__init__(node, multiprocessing)

        self.left_img_contours:Contours = None
        self.right_img_contours:Contours = None

        self.buoys:BuoyArray = None

    def _update_debug_pub_sizes(self, num:int):
        if self.node is None:
            return
        self.node.update_debug_pub_sizes(num)
    
    def run(self):
        self._debug('executing')

        if (self.left_img_contours is None or self.left_cam_info is None 
            or self.right_img_contours is None or self.right_cam_info is None):
            self._debug('something is none')
            return

        self._find_rect_maps()
        if self._left_rect_map is None or self._right_rect_map is None:
            return
        self._debug('got rect maps')

        if (len(self.left_img_contours.contour_offsets) != 
            len(self.right_img_contours.contour_offsets)):
            return
        
        self._debug('got cv2 contours')
        
        self.buoys = self._find_buoys()

        self._debug_pcd(self.buoys)
    
    def _find_buoys(self):

        contours = [
            unflatten_contours(self.left_img_contours.contour_points, 
                self.left_img_contours.contour_offsets),
            unflatten_contours(self.right_img_contours.contour_points, 
                self.right_img_contours.contour_offsets),
        ]
        contour_colors = [
            self.left_img_contours.contour_colors,
            self.right_img_contours.contour_colors
        ]
        self._sort_contours(contours, contour_colors)

        self._debug(f'sorted colors: {contour_colors}')

        buoy_pairs = list()
        for cnt_num in range(len(self.left_img_contours.contour_offsets)):
            pair = list()
            for img_num in range(2):
                blank = np.zeros((self.left_cam_info.height, self.left_cam_info.width))
                filled = cv2.drawContours(blank, contours[img_num], cnt_num, 255, 1).astype('uint8')
                pair.append(filled)
            buoy_pairs.append(pair)

        if len(buoy_pairs) == 0:
            self._debug('no contours')
            return BuoyArray()
        
        self._update_debug_pub_sizes(len(buoy_pairs))

        self._debug(f'pub length: {len(buoy_pairs)}')

        if self._multiprocessing:
            self._find_poses_multiprocessing(buoy_pairs)
        else:
            self._find_poses_singleprocessing(buoy_pairs)
        
        buoys = BuoyArray()
        for i in range(len(self._points) // 2):
            buoy = Buoy(location=Point(x=self._points[i * 2], y=self._points[(i * 2) + 1]),
                color=contour_colors[0][i])
            buoys.buoys.append(buoy)
        
        return buoys
    
    def _find_poses_singleprocessing(self, buoy_pairs:list):
        self._points = list(0.0 for _ in range(len(buoy_pairs) * 2))

        for i, pair in enumerate(buoy_pairs):
            self._find_buoy_pose(pair[0], pair[1], i) 
    
    def _find_poses_multiprocessing(self, buoy_pairs:list):
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

        self._debug(f'cam_transform x: {self.cam_transform[0][0]}')

        x, y = img_points_to_physical_xy(midpoints[0], midpoints[1], 
            self.left_cam_info.k[0], self.right_cam_info.k[0], 
            (len(left_img_rect) // 2, len(left_img_rect[0]) // 2),
            abs(self.cam_transform[0][0])
        )

        points_base_index = buoy_index * 2
        self._points[points_base_index] = x
        self._points[points_base_index + 1] = y

    def _debug_pcd(self, buoys:BuoyArray):
        pcd = PointCloud2()
        pcd.header.frame_id = 'wamv/lidar_wamv_link'
        pcd_points = list()
        for buoy in buoys.buoys:
            pcd_points.append([buoy.location.x, buoy.location.y, 0])
        
        pcd = create_cloud_xyz32(pcd.header, pcd_points)

        self._debug_pub('/perception/stereo/debug/points', pcd)
