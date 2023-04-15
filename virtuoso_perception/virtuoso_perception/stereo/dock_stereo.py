from .stereo import Stereo
from virtuoso_msgs.msg import Contours
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from virtuoso_perception.utils.pointcloud import create_cloud_xyz32
from .utils import unflatten_contours, img_points_to_physical_xy
from typing import List
import numpy as np
import cv2
from .pixel_matcher import PixelMatcher

class DockStereo(Stereo):

    def __init__(self, node, multiprocessing:bool):
        super().__init__(node, multiprocessing)

        self.left_img_contours:Contours = None
        self.right_img_contours:Contours = None

        self.end_points:List[Point] = None
    
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
        self._debug('got contours')

        self.end_points = self._find_end_points()

        self._debug_pcd(self.end_points)
    
    def _find_end_points(self):

        contours = [
            unflatten_contours(self.left_img_contours.contour_points, 
                self.left_img_contours.contour_offsets),
            unflatten_contours(self.right_img_contours.contour_points,
                self.right_img_contours.contour_offsets)
        ]
        contour_colors = [
            self.left_img_contours.contour_colors,
            self.right_img_contours.contour_colors
        ]
        self._sort_contours(contours, contour_colors)

        self._debug(f'sorted colors: {contour_colors}')

        if len(contour_colors[0]) < 2:
            self._debug('Less than 2 contours')
            return list()
        
        used_colors = set()
        for i in range(len(contour_colors[0])):
            if contour_colors[0][i] != contour_colors[1][i]:
                self._debug('Contour colors not matching between cameras')
                return list()
            if i == 0: 
                used_colors.add(contour_colors[0][i])
                continue
            if (contour_colors[0][i] != contour_colors[0][i - 1] and 
                contour_colors[0][i] in used_colors):
                self._debug('Contour colors in wrong order')
                return list()
            used_colors.add(contour_colors[0][i])
        
        mid_index = (len(contours[0]) // 2) - 1
        
        left_cam_left = np.zeros((self.left_cam_info.height, self.left_cam_info.width))
        left_cam_left = cv2.drawContours(left_cam_left, contours[0], mid_index, 255, 1).astype('uint8')

        left_cam_right = np.zeros((self.left_cam_info.height, self.left_cam_info.width))
        left_cam_right = cv2.drawContours(left_cam_right, contours[0], mid_index + 1, 255, 1).astype('uint8')

        right_cam_left = np.zeros((self.left_cam_info.height, self.left_cam_info.width))
        right_cam_left = cv2.drawContours(right_cam_left, contours[1], mid_index, 255, 1).astype('uint8')

        right_cam_right = np.zeros((self.left_cam_info.height, self.left_cam_info.width))
        right_cam_right = cv2.drawContours(right_cam_right, contours[1], mid_index + 1, 255, 1).astype('uint8')

        left = self._find_point(left_cam_left, right_cam_left, 'left')

        right = self._find_point(left_cam_right, right_cam_right, 'right')

        return [left, right]
    
    def _find_point(self, left_img, right_img, side):

        left_img_rect = cv2.remap(left_img, *self._left_rect_map, cv2.INTER_LANCZOS4)
        right_img_rect = cv2.remap(right_img, *self._right_rect_map, cv2.INTER_LANCZOS4)

        self._debug_pub(f'/perception/dock_stereo/left_cam_{side}', 
            self._cv_bridge.cv2_to_imgmsg(left_img_rect, encoding='mono8'))
        self._debug_pub(f'/perception/dock_stereo/right_cam_{side}', 
            self._cv_bridge.cv2_to_imgmsg(right_img_rect, encoding='mono8'))
        
        midpoints = PixelMatcher.midpoints(left_img_rect, right_img_rect)

        self._debug(f'midpoints: {midpoints}')

        self._debug(f'cam_transform x: {self.cam_transform[0][0]}')

        x, y = img_points_to_physical_xy(midpoints[0], midpoints[1], 
            self.left_cam_info.k[0], self.right_cam_info.k[0],
            (len(left_img_rect) // 2, len(left_img_rect[0]) // 2),
            abs(self.cam_transform[0][0])
        )

        return Point(x=x,y=y)
    
    def _debug_pcd(self, end_points):
        pcd = PointCloud2()
        pcd.header.frame_id = 'wamv/lidar_wamv_link'
        pcd_points = list()
        for pt in end_points:
            pcd_points.append([pt.x, pt.y, 0])
        
        pcd = create_cloud_xyz32(pcd.header, pcd_points)

        self._debug_pub('/perception/stereo/debug/points', pcd)