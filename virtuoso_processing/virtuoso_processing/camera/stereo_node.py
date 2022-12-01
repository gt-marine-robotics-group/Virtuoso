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
import open3d

class StereoNode(Node):

    def __init__(self):
        super().__init__('processing_stereo')

        # self.image1_sub = self.create_subscription(Image, '/processing/image/grayscaled1', 
        #     self.image1_callback, 10)
        self.image1_sub = self.create_subscription(Image, 
            '/wamv/sensors/cameras/front_left_camera/image_raw', 
            self.image1_callback, 10)
        self.cam_info1_sub = self.create_subscription(CameraInfo, 
            '/wamv/sensors/cameras/front_left_camera/camera_info', self.cam_info1_callback, 10)
        
        # self.image2_sub = self.create_subscription(Image, '/processing/image/grayscaled2',
        #     self.image2_callback, 10)
        self.image2_sub = self.create_subscription(Image,
            '/wamv/sensors/cameras/front_right_camera/image_raw',
            self.image2_callback, 10)
        self.cam_info2_sub = self.create_subscription(CameraInfo,
            '/wamv/sensors/cameras/front_right_camera/camera_info', self.cam_info2_callback, 10)
        
        self.debug_image_pub = self.create_publisher(Image, '/processing/stereo/debug', 10)

        self.image1:Image = None 
        self.cam_info1:CameraInfo = None

        self.image2:Image = None
        self.cam_info2:CameraInfo = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.stop = False

        self.create_timer(0.1, self.execute)
    
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
    
    def execute(self):

        if self.stop:
            return

        if (self.image1 is None or self.cam_info1 is None 
            or self.image2 is None or self.cam_info2 is None):
            self.get_logger().info('something is none')
            return
        
        bgr_image1 = CvBridge().imgmsg_to_cv2(self.image1, desired_encoding='bgr8')
        bgr_image2 = CvBridge().imgmsg_to_cv2(self.image2, desired_encoding='bgr8')

        image_size = (self.cam_info1.width, self.cam_info1.height)

        trans = self.get_c2_to_c1_transform()
        if trans is None: return
        
        # self.stop = True

        c1_matrix, c1_distortion = self.find_intrinsic(self.cam_info1)
        c2_matrix, c2_distortion = self.find_intrinsic(self.cam_info2)

        # undistort_map1 = cv2.initUndistortRectifyMap(c1_matrix, c1_distortion, 
        #     np.eye(3), c1_matrix, image_size, 
        #     cv2.CV_32F)
        # undistort_map2 = cv2.initUndistortRectifyMap(c2_matrix, c2_distortion,
        #     np.eye(3), c2_matrix, image_size,
        #     cv2.CV_32F)
        
        # img_undist1 = cv2.remap(bgr_image1, undistort_map1[0], undistort_map1[1], 
        #     cv2.INTER_LANCZOS4)
        # img_undist2 = cv2.remap(bgr_image2, undistort_map2[0], undistort_map2[1],
        #     cv2.INTER_LANCZOS4)

        # self.pub_debug(img_undist2)
        
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
        
        rect_map1 = cv2.initUndistortRectifyMap(c1_matrix, c1_distortion, 
            R1, P1, image_size, cv2.CV_32F)
        
        rect_map2 = cv2.initUndistortRectifyMap(c2_matrix, c2_distortion, 
            R2, P2, image_size, cv2.CV_32F)
        
        img_rect1 = cv2.remap(bgr_image1, *rect_map1, cv2.INTER_LANCZOS4)
        img_rect2 = cv2.remap(bgr_image2, *rect_map2, cv2.INTER_LANCZOS4)

        # self.get_logger().info(str(np.shape(img_rect1)))

        self.pub_debug(img_rect2)

        self.stop = True

        matcher = StereoMatcherSGBM(self)

        disparity = matcher.match(img_rect1, img_rect2)

        # self.plot(disparity)
        # plt.imshow(disparity)

        cv2.imshow('depth', disparity)
        cv2.waitKey(0)
        return
        # self.get_logger().info(str(disparity))

        pointcloud = matcher.reconstruct(disparity, img_rect1, Q)

        self.get_logger().info(str(pointcloud.points))

        pointcloud, _ = pointcloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=1.5
        )

        self.get_logger().info(str(pointcloud.points))

        # self.display_pcd(pointcloud)

    
    def plot(self, map):
        xy = np.dstack(np.meshgrid(range(self.cam_info1.width), range(self.cam_info1.height)))
        ud_map1_abs = np.linalg.norm(
            np.dstack(map) - xy, axis=2
        )
        plt.imshow(ud_map1_abs)
        plt.colorbar()
        plt.show()
    
    def display_pcd(self, pointcloud):
        origin_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.10, origin=[0, 0, 0]
        )

        open3d.visualization.draw_geometries([pointcloud, origin_frame])
    
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