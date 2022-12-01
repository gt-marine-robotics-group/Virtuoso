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

class StereoNode(Node):

    def __init__(self):
        super().__init__('processing_stereo')

        self.image1_sub = self.create_subscription(Image, '/processing/image/grayscaled1', 
            self.image1_callback, 10)
        self.cam_info1_sub = self.create_subscription(CameraInfo, 
            '/wamv/sensors/cameras/front_left_camera/camera_info', self.cam_info1_callback, 10)
        
        self.image2_sub = self.create_subscription(Image, '/processing/image/grayscaled2',
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
        trans = None
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
        # self.get_logger().info(str(self.cam_info))

        if self.stop:
            return

        if (self.image1 is None or self.cam_info1 is None 
            or self.image2 is None or self.cam_info2 is None):
            self.get_logger().info('something is none')
            return
        
        # self.get_logger().info(str(self.image1.header.frame_id))
        # self.get_logger().info(str(self.image2.header.frame_id))
        
        bgr_image1 = CvBridge().imgmsg_to_cv2(self.image1, desired_encoding='mono8')
        bgr_image2 = CvBridge().imgmsg_to_cv2(self.image2, desired_encoding='mono8')

        trans = self.get_c2_to_c1_transform()
        # self.get_logger().info(str(trans))
        if trans is None: return
        
        # self.stop = True

        c1_matrix, c1_distortion = self.find_intrinsic(self.cam_info1)
        c2_matrix, c2_distortion = self.find_intrinsic(self.cam_info2)

        undistort_map1 = cv2.initUndistortRectifyMap(c1_matrix, c1_distortion, 
            np.eye(3), c1_matrix, (self.cam_info1.width, self.cam_info1.height), 
            cv2.CV_32F)
        undistort_map2 = cv2.initUndistortRectifyMap(c2_matrix, c2_distortion,
            np.eye(3), c2_matrix, (self.cam_info2.width, self.cam_info2.height),
            cv2.CV_32F)
        
        img_undist1 = cv2.remap(bgr_image1, undistort_map1[0], undistort_map1[1], 
            cv2.INTER_LANCZOS4)
        img_undist2 = cv2.remap(bgr_image2, undistort_map2[0], undistort_map2[1],
            cv2.INTER_LANCZOS4)
        
        self.pub_debug(img_undist2)
    
    def plot(self, map):
        xy = np.dstack(np.meshgrid(range(self.cam_info1.width), range(self.cam_info1.height)))
        ud_map1_abs = np.linalg.norm(
            np.dstack(map) - xy, axis=2
        )
        plt.imshow(ud_map1_abs)
        plt.colorbar()
        plt.show()
    
    def pub_debug(self, bgr_image):
        msg = CvBridge().cv2_to_imgmsg(bgr_image, encoding='mono8')
        self.debug_image_pub.publish(msg)


def main(args=None):
    
    rclpy.init(args=args)

    sub = StereoNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()