import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

class StereoNode(Node):

    def __init__(self):
        super().__init__('processing_stereo')

        self.image1_sub = self.create_subscription(Image, '/processing/image/downscaled', 
            self.image1_callback, 10)
        self.cam_info1_sub = self.create_subscription(CameraInfo, 
            '/wamv/sensors/cameras/front_left_camera/camera_info', self.cam_info1_callback, 10)
        
        self.debug_image_pub = self.create_publisher(Image, '/processing/stereo/debug', 10)

        self.image1:Image = None 
        self.cam_info1:CameraInfo = None

        self.stop = False

        self.create_timer(0.1, self.execute)
    
    def image1_callback(self, msg:Image):
        self.image1 = msg
    
    def cam_info1_callback(self, msg:CameraInfo):
        self.cam_info1 = msg
    
    def execute(self):
        # self.get_logger().info(str(self.cam_info))

        if self.stop:
            return

        if self.image1 is None or self.cam_info1 is None:
            self.get_logger().info('something is none')
            return
        
        bgr_image = CvBridge().imgmsg_to_cv2(self.image1, desired_encoding='bgr8')
        
        # self.stop = True

        k = self.cam_info1.k

        camera_matrix = np.array([
            [k[0], 0, k[2]],
            [0, k[4], k[5]],
            [0, 0, 1]
        ]) 
        
        camera_distortion = np.array(self.cam_info1.d)

        undistort_map = cv2.initUndistortRectifyMap(camera_matrix, camera_distortion, 
            np.eye(3), camera_matrix, (self.cam_info1.width, self.cam_info1.height), 
            cv2.CV_32F)

        self.get_logger().info('RUN')
        # self.plot(undistort_map)

        img_undist = cv2.remap(bgr_image, undistort_map[0], undistort_map[1], 
            cv2.INTER_LANCZOS4)
        
        self.pub_debug(img_undist)
    
    def plot(self, map):
        xy = np.dstack(np.meshgrid(range(self.cam_info1.width), range(self.cam_info1.height)))
        ud_map1_abs = np.linalg.norm(
            np.dstack(map) - xy, axis=2
        )
        plt.imshow(ud_map1_abs)
        plt.colorbar()
        plt.show()
    
    def pub_debug(self, bgr_image):
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