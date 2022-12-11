import rclpy
from rclpy.node import Node
from .stereo import Stereo
from virtuoso_msgs.msg import Contours
from sensor_msgs.msg import CameraInfo, PointCloud2, Image
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from .utils import tf_transform_to_cv2_transform

class StereoNode(Node):

    def __init__(self):
        super().__init__('perception_stereo')

        self.declare_parameters(namespace='', parameters=[
            ('base_topics', []),
            ('frames', [])
        ])

        self.frames = self.get_parameter('frames').value
        base_topics = self.get_parameter('base_topics').value

        self.left_buoy_contours_sub = self.create_subscription(Contours,
            f'{base_topics[0]}/buoy_filter', self.left_buoy_contours_callback, 10)
        self.left_cam_info_sub = self.create_subscription(CameraInfo, 
            f'{base_topics[0]}/camera_info', self.left_cam_info_callback, 10)
        
        self.right_buoy_contours_sub = self.create_subscription(Contours,
            f'{base_topics[1]}/buoy_filter', self.right_buoy_contours_callback, 10)
        self.right_cam_info_sub = self.create_subscription(CameraInfo,
            f'{base_topics[1]}/camera_info', self.right_cam_info_callback, 10)
        
        # self.pcd_pub = self.create_publisher(PointCloud2, '/perception/stereo/points', 10)

        self.debug_pubs = {
            '/perception/stereo/debug/left_cam/contoured_buoy': [
                self.create_publisher(Image, '/perception/stereo/debug/left_cam/contoured_buoy1', 10)
            ],
            '/perception/stereo/debug/right_cam/contoured_buoy': [
                self.create_publisher(Image, '/perception/stereo/debug/right_cam/contoured_buoy1', 10)
            ],
            '/perception/stereo/debug/left_cam/rectified_buoy': [
                self.create_publisher(Image, '/perception/stereo/debug/left_cam/rectified1', 10)
            ],
            '/perception/stereo/debug/right_cam/rectified_buoy': [
                self.create_publisher(Image, '/perception/stereo/debug/right_cam/rectified1', 10)
            ],
            '/perception/stereo/debug/points': [
                self.create_publisher(PointCloud2, '/perception/stereo/debug/points', 10)
            ]
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.stereo = Stereo(node=self, multiprocessing=True)

        self.create_timer(1.0, self.execute)
    
    def update_debug_pub_sizes(self, size:int):
        for base_name, pubs in self.debug_pubs.items():
            if not isinstance(pubs, (list)): continue
            curr_i = len(pubs)
            while curr_i < size:
                pubs.append(
                    self.create_publisher(pubs[0].msg_type, f'{base_name}{curr_i + 1}', 10)
                )
                curr_i += 1
    
    def debug_pub_indexed(self, base:str, num:int, msg):
        self.debug_pubs[base][num].publish(msg)
    
    def left_buoy_contours_callback(self, msg:Contours):
        self.stereo.left_buoy_img_contours = msg
    
    def right_buoy_contours_callback(self, msg:Contours):
        self.stereo.right_buoy_img_contours = msg

    def left_cam_info_callback(self, msg:CameraInfo):
        self.stereo.left_cam_info = msg
    
    def right_cam_info_callback(self, msg:CameraInfo):
         self.stereo.right_cam_info = msg
        
    def find_cam_transform(self):
        trans:TransformStamped = None
        try:
            trans = self.tf_buffer.lookup_transform(
                self.frames[0],
                self.frames[1],
                Time()
            )
        except:
            self.get_logger().info('Transform failed')
            return
        
        cv2_trans = tf_transform_to_cv2_transform(trans)
        
        self.stereo.cam_transform = cv2_trans
        
    def execute(self):
        if self.stereo.cam_transform is None:
            self.find_cam_transform()
            return
        
        self.stereo.run()

        if self.stereo.pcd is None:
            return
        
        # self.pcd_pub.publish(self.stereo.pcd)     
        self.debug_pubs['/perception/stereo/debug/points'][0].publish(self.stereo.pcd)


def main(args=None):
    
    rclpy.init(args=args)

    node = StereoNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()