import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from scipy.spatial.transform import Rotation
from .stereo_matcher import StereoMatcherSGBM
from sensor_msgs.msg import PointCloud2
from virtuoso_processing.utils.pointcloud import create_cloud_xyz32
import time

class StereoNode(Node):

    def __init__(self):
        super().__init__('processing_stereo')

        self.image1_sub = self.create_subscription(Image, '/processing/image/grayscaled1', 
            self.image1_callback, 10)
        # self.image1_sub = self.create_subscription(Image, 
        #     '/wamv/sensors/cameras/front_left_camera/image_raw', 
        #     self.image1_callback, 10)
        self.cam_info1_sub = self.create_subscription(CameraInfo, 
            '/wamv/sensors/cameras/front_left_camera/camera_info', self.cam_info1_callback, 10)
        
        self.image2_sub = self.create_subscription(Image, '/processing/image/grayscaled2',
            self.image2_callback, 10)
        # self.image2_sub = self.create_subscription(Image,
        #     '/wamv/sensors/cameras/front_right_camera/image_raw',
        #     self.image2_callback, 10)
        self.cam_info2_sub = self.create_subscription(CameraInfo,
            '/wamv/sensors/cameras/front_right_camera/camera_info', self.cam_info2_callback, 10)
        
        self.debug_image_pub = self.create_publisher(Image, '/processing/stereo/debug', 10)

        self.pcd_pub = self.create_publisher(PointCloud2, '/processing/stereo/points', 10)

        self.image1:Image = None 
        self.cam_info1:CameraInfo = None

        self.image2:Image = None
        self.cam_info2:CameraInfo = None

        self.rect_map1 = None
        self.rect_map2 = None
        self.Q = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.stop = False
        
        self.matcher = StereoMatcherSGBM(self)

        self.create_timer(1.0, self.execute)
    
    def image1_callback(self, msg:Image):
        self.image1 = msg
    
    def cam_info1_callback(self, msg:CameraInfo):
        self.cam_info1 = msg
    
    def image2_callback(self, msg:Image):
        self.image2 = msg
    
    def cam_info2_callback(self, msg:CameraInfo):
        self.cam_info2 = msg
    
    def get_c2_to_c1_transform(self):
        trans:TransformStamped = None
        try:
            trans = self.tf_buffer.lookup_transform(
                'wamv/front_left_camera_link_optical',
                'wamv/front_right_camera_link_optical',
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
        
        # self.get_logger().info(str(c2_trans))

        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(c1_matrix, c1_distortion,
            c2_matrix, c2_distortion, image_size, c2_rodrigues, c2_trans,
            cv2.CALIB_ZERO_DISPARITY)
        
        self.Q = Q
        
        self.rect_map1 = cv2.initUndistortRectifyMap(c1_matrix, c1_distortion, 
            R1, P1, image_size, cv2.CV_32F)
        
        self.rect_map2 = cv2.initUndistortRectifyMap(c2_matrix, c2_distortion, 
            R2, P2, image_size, cv2.CV_32F)
    
    def execute(self):
        self.get_logger().info('executing')

        if self.stop:
            return

        if (self.image1 is None or self.cam_info1 is None 
            or self.image2 is None or self.cam_info2 is None):
            self.get_logger().info('something is none')
            return
        
        try:
            bgr_image1 = CvBridge().imgmsg_to_cv2(self.image1, desired_encoding='mono8')
            bgr_image2 = CvBridge().imgmsg_to_cv2(self.image2, desired_encoding='mono8')
        except:
            self.get_logger().info('ERROR CONVERTING ROS TO CV2 IMAGE')
            return
        self.get_logger().info('got cv2 images')

        self.find_rect_maps()
        if self.rect_map1 is None or self.rect_map2 is None:
            return
        self.get_logger().info('got rect maps')
        
        try:
            img_rect1 = cv2.remap(bgr_image1, *self.rect_map1, cv2.INTER_LANCZOS4)
            img_rect2 = cv2.remap(bgr_image2, *self.rect_map2, cv2.INTER_LANCZOS4)
        except:
            self.get_logger().info('REMAPPING ERROR')
            return
        self.get_logger().info('got image rects')

        try:
            disparity = self.matcher.match(img_rect1, img_rect2)
            # disparity = matcher.match(cv2.cvtColor(img_rect1, cv2.COLOR_BGR2GRAY),
            #     cv2.cvtColor(img_rect2, cv2.COLOR_BGR2GRAY))
        except:
            self.get_logger().info('DISPARITY ERROR')
            return
        self.get_logger().info('got disparity')

        # cv2.imshow('depth', disparity)
        # cv2.waitKey(0)

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

    def pub_pointcloud(self, cv_pcd):
        pcd = PointCloud2()
        pcd.height = 1
        pcd.width = 0
        pcd.header.frame_id = 'camera'

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
        # msg = CvBridge().cv2_to_imgmsg(bgr_image, encoding='mono8')
        msg = CvBridge().cv2_to_imgmsg(bgr_image, encoding='rgb8')
        self.debug_image_pub.publish(msg)


def main(args=None):
    
    rclpy.init(args=args)

    sub = StereoNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()