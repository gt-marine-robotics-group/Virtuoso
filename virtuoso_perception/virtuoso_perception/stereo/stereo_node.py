import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import numpy as np
import cv2
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from virtuoso_processing.utils.pointcloud import create_cloud_xyz32
import time
from virtuoso_msgs.msg import Contours
import math
from typing import List
from multiprocessing import Process, Array, Pool

class StereoNode(Node):

    def __init__(self):
        super().__init__('perception_stereo')

        self.declare_parameters(namespace='', parameters=[
            ('base_topics', []),
            ('frames', [])
        ])

        base_topics = self.get_parameter('base_topics').value
        self.frames = self.get_parameter('frames').value

        self.filtered1_sub = self.create_subscription(Contours,
            f'{base_topics[0]}/buoy_filter', self.filtered1_callback, 10)
        self.cam_info1_sub = self.create_subscription(CameraInfo, 
            f'{base_topics[0]}/camera_info', self.cam_info1_callback, 10)
        
        self.filtered2_sub = self.create_subscription(Contours,
            f'{base_topics[1]}/buoy_filter', self.filtered2_callback, 10)
        self.cam_info2_sub = self.create_subscription(CameraInfo,
            f'{base_topics[1]}/camera_info', self.cam_info2_callback, 10)
        
        self.debug_disparity_pubs = [
            self.create_publisher(Image, '/perception/stereo/debug/disparity1', 10)
        ]
        self.debug_received_img_pubs = [
            self.create_publisher(Image, '/perception/stereo/debug/received_img1', 10),
            self.create_publisher(Image, '/perception/stereo/debug/received_img2', 10)
        ]

        # RENAME TO LEFT_CAM
        self.debug_contoured_buoy_cam1_pub = [
            self.create_publisher(Image, '/perception/stereo/debug/cam1/contoured_buoy1', 10)
        ]

        # RENAME TO RIGHT_CAM
        self.debug_contoured_buoy_cam2_pub = [
            self.create_publisher(Image, '/perception/stereo/debug/cam2/contoured_buoy1', 10)
        ]

        # RENAME TO LEFT_CAM
        self.debug_rectified_cam1_pub = [
            self.create_publisher(Image, '/perception/stereo/debug/cam1/rectified1', 10)
        ]

        # RENAME TO RIGHT_CAM
        self.debug_rectified_cam2_pub = [
            self.create_publisher(Image, '/perception/stereo/debug/cam2/rectified1', 10)
        ]

        self.pcd_pub = self.create_publisher(PointCloud2, '/perception/stereo/points', 10)
        # self.pcd_pubs = [
        #     self.create_publisher(PointCloud2, '/perception/stereo/points1', 10)
        # ]

        # self.image1:Image = None 
        self.buoy_filtered1:Contours = None
        self.cam_info1:CameraInfo = None

        # self.image2:Image = None
        self.buoy_filtered2:Contours = None
        self.cam_info2:CameraInfo = None

        self.rect_map1 = None
        self.rect_map2 = None
        self.Q = None

        self.points = list()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cv_bridge = CvBridge()

        self.stop = False
        
        self.matcher = cv2.StereoBM_create(
            numDisparities=64,
            blockSize=101
        )
        self.matcher.setSpeckleWindowSize(10)

        window_size = 15
        # self.matcher = cv2.StereoSGBM_create(
            # minDisparity=0,
            # numDisparities=128,
            # speckleWindowSize=10,
            # speckleRange=1,
            # disp12MaxDiff=1,
            # uniquenessRatio=5

            # disp12MaxDiff=-1,
            # blockSize=11,
            # P1=0,
            # P2=1,
            # uniquenessRatio=15,
            # speckleRange=128

            # blockSize=window_size,
            # P1=9 * 3 * window_size,
            # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
            # P2=128 * 3 * window_size,
            # P1=1,
            # P2=2,
            # disp12MaxDiff=12,
            # disp12MaxDiff=-1,
        #     uniquenessRatio=40,
        #     speckleWindowSize=50,
        #     speckleRange=32,
        #     preFilterCap=63,
        #     mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        # )

        self.create_timer(1.0, self.execute)
    
    def filtered1_callback(self, msg:Contours):
        self.buoy_filtered1 = msg
    
    def cam_info1_callback(self, msg:CameraInfo):
        self.cam_info1 = msg
    
    def filtered2_callback(self, msg:Contours):
        self.buoy_filtered2 = msg
    
    def cam_info2_callback(self, msg:CameraInfo):
        self.cam_info2 = msg
    
    def update_publishers_size(self, publishers:List[Publisher], base:str, num:int):
        curr_i = len(publishers)
        while curr_i < num:
            publishers.append(
                self.create_publisher(publishers[0].msg_type, f'{base}{curr_i + 1}', 10)
            )
            curr_i += 1
    
    def get_c2_to_c1_transform(self):
        trans:TransformStamped = None
        try:
            trans = self.tf_buffer.lookup_transform(
                self.frames[0],
                self.frames[1],
                Time()
            )
        except:
            self.get_logger().info('Transform failed')
        
        return trans
    
    def find_intrinsic(self, cam_info:CameraInfo):
        k = cam_info.k

        camera_matrix = np.array([
            [k[0], 0, k[2]],
            [0, k[4], k[5]],
            [0, 0, 1]
        ]) 
        
        camera_distortion = np.array(cam_info.d)

        return camera_matrix, camera_distortion
    
    def find_rect_maps(self):
        if not self.rect_map1 is None and not self.rect_map2 is None:
            return

        image_size = (self.cam_info1.width, self.cam_info1.height)
        # image_size = (self.cam_info1.height, self.cam_info1.width)

        trans = self.get_c2_to_c1_transform()
        if trans is None: return

        c1_matrix, c1_distortion = self.find_intrinsic(self.cam_info1)
        c2_matrix, c2_distortion = self.find_intrinsic(self.cam_info2)
        
        c1_pose = Pose(position=Point(x=0.0, y=0.0, z=0.0), 
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        
        trans_vector = trans.transform.translation
        # For some reason the transformation from right cam to left cam is x = 0.2,
        # should probably be y = 0.2 but may have something to do with one of the camera's
        # orientation.
        c2_pose = Pose(position=Point(x=trans_vector.x, y=trans_vector.y, z=trans_vector.z),
            orientation=trans.transform.rotation)
        c2_rodrigues = Rotation.from_quat([c2_pose.orientation.x, c2_pose.orientation.y,
            c2_pose.orientation.z, c2_pose.orientation.w]).as_mrp()
        c2_trans = np.array([float(c2_pose.position.x), float(c2_pose.position.y),
            float(c2_pose.position.z)])
        
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(c1_matrix, c1_distortion,
            c2_matrix, c2_distortion, image_size, c2_rodrigues, c2_trans,
            cv2.CALIB_ZERO_DISPARITY)
        
        self.Q = Q
        
        self.rect_map1 = cv2.initUndistortRectifyMap(c1_matrix, c1_distortion, 
            R1, P1, image_size, cv2.CV_32F)
        
        self.rect_map2 = cv2.initUndistortRectifyMap(c2_matrix, c2_distortion, 
            R2, P2, image_size, cv2.CV_32F)
    
    def unflatten_contours(self, flat_contours:list, contour_offsets:list):
        contours = np.empty((len(contour_offsets),), dtype=object)

        for i, offset in enumerate(contour_offsets):
            if i == len(contour_offsets) - 1:
                final_i = len(flat_contours)
            else:
                final_i = contour_offsets[i + 1]
            
            contour = np.empty(((final_i - offset) // 2, 1, 2), dtype='int64')
            flat_i = offset
            count = 0
            while flat_i < final_i:
                contour[count,0,0] = flat_contours[flat_i]
                contour[count,0,1] = flat_contours[flat_i + 1]
                count += 1
                flat_i += 2

            contours[i] = contour
        
        return contours
    
    def find_left_cam_x_angle(self, point:tuple, f:float, center:tuple):
        if point[1] > center[1]:
            return math.atan(f / (point[1] - center[1]))

        if point[1] < center[1]:
            return math.atan((center[1] - point[1]) / f) + (math.pi / 2)

        return math.pi / 2
    
    def find_right_cam_x_angle(self, point:tuple, f:float, center:tuple):
        if point[1] > center[1]:
            return math.atan((point[1] - center[1]) / f) + (math.pi / 2)
        
        if point[1] < center[1]:
            return math.atan(f / (center[1] - point[1]))
        
        return math.pi / 2
    
    # fx1 and fx2 should be the same
    def find_object_xy(self, mid1:tuple, mid2:tuple, fx1:float, fx2:float, center:tuple,
        cam_separation=0.2):
        left_x_theta = self.find_left_cam_x_angle(mid1, fx1, center)
        right_x_theta = self.find_right_cam_x_angle(mid2, fx2, center)

        self.get_logger().info(f'left theta: {left_x_theta * 180 / math.pi}')
        self.get_logger().info(f'right theta: {right_x_theta * 180 / math.pi}')

        s_theta = math.pi - left_x_theta - right_x_theta

        left_hyp = math.sin(right_x_theta) * cam_separation / s_theta

        object_x = left_hyp * math.sin(left_x_theta)

        object_y = math.sqrt(left_hyp**2 - object_x**2)
        if mid1[1] > center[1]: object_y *= -1

        self.get_logger().info(f'object x: {object_x}')
        self.get_logger().info(f'object y: {object_y}')

        return object_x, object_y
    
    def execute(self):
        self.get_logger().info('executing')

        if self.stop:
            return

        if (self.buoy_filtered1 is None or self.cam_info1 is None 
            or self.buoy_filtered2 is None or self.cam_info2 is None):
            self.get_logger().info('something is none')
            return

        if (len(self.buoy_filtered1.contour_offsets) != 
            len(self.buoy_filtered2.contour_offsets)):
            return

        # try:
        #     mono_image1 = self.cv_bridge.imgmsg_to_cv2(self.buoy_filtered1.image, desired_encoding='mono8')
        #     mono_image2 = self.cv_bridge.imgmsg_to_cv2(self.buoy_filtered2.image, desired_encoding='mono8')
        # except:
        #     self.get_logger().info('ERROR CONVERTING ROS TO CV2 IMAGE')
        #     return
        self.get_logger().info('got cv2 images')

        # self.debug_received_img_pubs[0].publish(
        #     self.cv_bridge.cv2_to_imgmsg(mono_image1, encoding='mono8')
        # )
        # self.debug_received_img_pubs[1].publish(
        #     self.cv_bridge.cv2_to_imgmsg(mono_image1, encoding='mono8')
        # )

        
        contours = [
            self.unflatten_contours(self.buoy_filtered1.contour_points, 
                self.buoy_filtered1.contour_offsets),
            self.unflatten_contours(self.buoy_filtered2.contour_points, 
                self.buoy_filtered2.contour_offsets)
        ]
        
        buoy_pairs = list()
        for cnt_num in range(len(self.buoy_filtered1.contour_offsets)):
            pair = list()
            for img_num in range(2):
                blank = np.zeros((self.cam_info1.height, self.cam_info1.width))
                filled = cv2.drawContours(blank, contours[img_num], cnt_num, 255, 1).astype('uint8')
                pair.append(filled)
                # pair.append(cv2.bitwise_and(filled, 
                #     mono_image1 if img_num == 0 else mono_image2))
            buoy_pairs.append(pair)
        
        if len(buoy_pairs) == 0:
            self.get_logger().info('no contours')
            return

        self.update_publishers_size(self.debug_contoured_buoy_cam1_pub, 
            '/perception/stereo/debug/cam1/contoured_buoy', len(buoy_pairs))
        self.update_publishers_size(self.debug_contoured_buoy_cam2_pub, 
            '/perception/stereo/debug/cam2/contoured_buoy', len(buoy_pairs))
        self.update_publishers_size(self.debug_rectified_cam1_pub, 
            '/perception/stereo/debug/cam1/rectified', len(buoy_pairs))
        self.update_publishers_size(self.debug_rectified_cam2_pub, 
            '/perception/stereo/debug/cam2/rectified', len(buoy_pairs))
        # self.update_publishers_size(self.pcd_pubs, '/perception/stereo/points', len(buoy_pairs))
        
        self.get_logger().info(f'pub length: {len(self.debug_contoured_buoy_cam1_pub)}')

        # self.run_stereo(buoy_pairs[0][0], buoy_pairs[0][1])
        # self.points = list(None for _ in range(len(buoy_pairs)))
        self.points = Array(typecode_or_type='d', size_or_initializer=len(buoy_pairs)*2)
        # processes = list()
        # for i, pair in enumerate(buoy_pairs):
        #     process = Process(target=self.run_stereo, args=(pair[0], pair[1], i))
        #     process.start()
        #     processes.append(process)
        #     self.run_stereo(pair[0], pair[1], i)

        # for p in processes:
        #     p.join()

        contours_remaining = True
        count = 0
        while contours_remaining:
            contours_remaining = False
            # run at most 8 processes
            pair_indexes = ((count * 8) + i for i in range(8))
            processes = list()
            for index in pair_indexes:
                if index >= len(buoy_pairs): break
                process = Process(target=self.run_stereo, 
                    args=(buoy_pairs[index][0], buoy_pairs[index][1], index))
                process.start()
                processes.append(process)
            else:
                contours_remaining = True
                count += 1
            
            for p in processes:
                p.join()
        
        pcd = PointCloud2()
        pcd.header.frame_id = 'wamv/lidar_wamv_link'
        pcd_points = list()
        for i in range(len(buoy_pairs)):
            pcd_points.append([self.points[i * 2], self.points[(i * 2) + 1], 0])
        
        pcd = create_cloud_xyz32(pcd.header, pcd_points)
        self.pcd_pub.publish(pcd)

        # pool = Pool()
        # for i, pair in enumerate(buoy_pairs):
        #     pool.apply_async(self.run_stereo, args=(pair[0], pair[1], i))
        
        # pool.close()
        # pool.join()
        
        self.get_logger().info(str(list(p for p in self.points)))
    
    def run_stereo(self, mono_image1, mono_image2, pub_index:int):
        self.get_logger().info(f'pub_index: {pub_index}')
        
        # self.debug_received_img_pub.publish(self.buoy_filtered1.image)
        # self.debug_received_img_pub.publish(CvBridge().cv2_to_imgmsg(mono_image1, encoding='mono8'))
        self.debug_contoured_buoy_cam1_pub[pub_index].publish(
            self.cv_bridge.cv2_to_imgmsg(mono_image1, encoding='mono8')
        )
        self.debug_contoured_buoy_cam2_pub[pub_index].publish(
            self.cv_bridge.cv2_to_imgmsg(mono_image2, encoding='mono8')
        )
        
        self.find_rect_maps()
        if self.rect_map1 is None or self.rect_map2 is None:
            return
        self.get_logger().info('got rect maps')
        
        try:
            img_rect1 = cv2.remap(mono_image1, *self.rect_map1, cv2.INTER_LANCZOS4)
            img_rect2 = cv2.remap(mono_image2, *self.rect_map2, cv2.INTER_LANCZOS4)
        except:
            self.get_logger().info('REMAPPING ERROR')
            return
        self.get_logger().info('got image rects')

        midpoints_indexes = [
            np.where(img_rect1 == 255),
            np.where(img_rect2 == 255)
        ]

        midpoint_counters = list(
            {'x-sum': 0, 'x-count': indexes[0].size, 'y-sum': 0, 'y-count': indexes[1].size} 
            for indexes in midpoints_indexes
        )

        for i in range(len(midpoints_indexes)):
            for j in range(midpoints_indexes[i][0].size):
                midpoint_counters[i]['x-sum'] += midpoints_indexes[i][1][j]
                midpoint_counters[i]['y-sum'] += midpoints_indexes[i][0][j]

        midpoints = list(
            (counter['y-sum'] // counter['y-count'], counter['x-sum'] // counter['x-count'])
            for counter in midpoint_counters
        )

        self.get_logger().info(f'midpoints: {midpoints}')

        center = (
            len(img_rect1) // 2, len(img_rect1[0]) // 2
        )
        x, y = self.find_object_xy(midpoints[0], midpoints[1], 
            self.cam_info1.k[0], self.cam_info2.k[0], center)

        self.debug_rectified_cam1_pub[pub_index].publish(
            self.cv_bridge.cv2_to_imgmsg(img_rect1, encoding='mono8')
        )
        self.debug_rectified_cam2_pub[pub_index].publish(
            self.cv_bridge.cv2_to_imgmsg(img_rect2, encoding='mono8')
        )

        self.pub_single_point_pcd((x, y), pub_index)

        return

        try:
            disparity = self.matcher.compute(img_rect1, img_rect2).astype(np.float32) / 16.0
            # disparity = self.matcher.compute(mono_image1, mono_image2).astype(np.float32) / 16.0
        except:
            self.get_logger().info('DISPARITY ERROR')
            return
        self.get_logger().info('got disparity')

        self.pub_debug(disparity)

        try:
            pointcloud = cv2.reprojectImageTo3D(disparity, self.Q)
        except:
            self.get_logger().info('3D REPROJECT ERROR')
            return
        self.get_logger().info('got cv2 pointcloud')

        try:
            self.pub_pointcloud(pointcloud)
        except Exception as e:
            self.get_logger().info(f'POINT CLOUD PUB ERROR: {e}')
            return
        self.get_logger().info('published')

    def pub_single_point_pcd(self, point:tuple, pub_index:int):
        pcd = PointCloud2()
        pcd.height = 1
        pcd.width = 1
        pcd.header.frame_id = 'wamv/lidar_wamv_link'

        pcd = create_cloud_xyz32(pcd.header, 
            [[point[0], point[1], 0]])

        self.get_logger().info('CREATED SINGLE POINT PCD')

        # self.pcd_pub.publish(pcd)
        # self.pcd_pubs[pub_index].publish(pcd)
        base = pub_index * 2
        self.points[base] = point[0]
        self.points[base + 1] = point[1]

    def pub_pointcloud(self, cv_pcd):
        pcd = PointCloud2()
        pcd.height = 1
        pcd.width = 0
        pcd.header.frame_id = 'wamv/lidar_wamv_link'

        # unfortunately this is the fastest solution I found
        start_time = time.time()
        points = list()
        for row in cv_pcd:
            for point in row:
                if point[2] > 0: continue
                # points.append(point)
                points.append([
                    -point[2],
                    point[0],
                    point[1]
                ])
                pcd.width += 1
        self.get_logger().info(f'exec time: {time.time() - start_time}')
        
        pcd = create_cloud_xyz32(pcd.header, points)

        self.get_logger().info('CREATED CLOUD')
        self.get_logger().info(str(pcd.width))
        self.pcd_pub.publish(pcd)
    
    def plot(self, map):
        xy = np.dstack(np.meshgrid(range(self.cam_info1.width), range(self.cam_info1.height)))
        ud_map1_abs = np.linalg.norm(
            np.dstack(map) - xy, axis=2
        )
        plt.imshow(ud_map1_abs)
        plt.colorbar()
        plt.show()
    
    def pub_debug(self, bgr_image):
        mono = cv2.convertScaleAbs(bgr_image, cv2.CV_8UC1, 100, 0.0)
        msg = CvBridge().cv2_to_imgmsg(mono, encoding='mono8')
        # msg = CvBridge().cv2_to_imgmsg(bgr_image, encoding='rgb8')
        # self.debug_image_pub.publish(msg)
        self.debug_disparity_pubs[0].publish(msg)


def main(args=None):
    
    rclpy.init(args=args)

    sub = StereoNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()