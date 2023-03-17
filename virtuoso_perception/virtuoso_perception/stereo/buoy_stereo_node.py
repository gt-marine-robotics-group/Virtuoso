import rclpy
from rclpy.node import Node
from .buoy_stereo import BuoyStereo
from virtuoso_msgs.msg import Contours, BuoyArray
from sensor_msgs.msg import CameraInfo, PointCloud2, Image
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from .utils import tf_transform_to_cv2_transform
from virtuoso_msgs.srv import ImageBuoyStereo, ImageBuoyFilter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

class BuoyStereoNode(Node):

    def __init__(self):
        super().__init__('perception_buoy_stereo')

        self.cb_group_1 = MutuallyExclusiveCallbackGroup()
        self.cb_group_2 = MutuallyExclusiveCallbackGroup()
        self.cb_group_3 = MutuallyExclusiveCallbackGroup()

        self.declare_parameters(namespace='', parameters=[
            ('base_topics', []),
            ('frames', []), 
            ('debug', False),
            ('multiprocessing', True)
        ])

        self.frames = self.get_parameter('frames').value
        base_topics = self.get_parameter('base_topics').value
        cams = list(topic[topic.rfind('/') + 1:] for topic in base_topics)

        self.left_cam_sub = self.create_subscription(Image,
            f'{base_topics[0]}/image_raw', self.left_cam_callback, 10)
        self.left_cam_info_sub = self.create_subscription(CameraInfo, 
            f'{base_topics[0]}/camera_info', self.left_cam_info_callback, 10)
        
        self.right_cam_sub = self.create_subscription(Image,
            f'{base_topics[1]}/image_raw', self.right_cam_callback, 10)
        self.right_cam_info_sub = self.create_subscription(CameraInfo,
            f'{base_topics[1]}/camera_info', self.right_cam_info_callback, 10)
        
        self.left_image = None
        self.right_image = None
        self.left_cam_info = None
        self.right_cam_info = None

        self.srv = self.create_service(ImageBuoyStereo, 'perception/image_buoy_stereo',
            self.srv_callback, callback_group=self.cb_group_1)
        
        self.left_cam_client = self.create_client(ImageBuoyFilter, 
            f'{cams[0]}/buoy_filter', callback_group=self.cb_group_2)
        self.right_cam_client = self.create_client(ImageBuoyFilter,
            f'{cams[1]}/buoy_filter', callback_group=self.cb_group_3)

        self.debug_pubs = {
            '/perception/stereo/debug/left_cam/contoured_buoy': [
                self.create_publisher(Image, '/perception/stereo/debug/left_cam/contoured_buoy1', 10)
            ],
            '/perception/stereo/debug/right_cam/contoured_buoy': [
                self.create_publisher(Image, '/perception/stereo/debug/right_cam/contoured_buoy1', 10)
            ],
            '/perception/stereo/debug/left_cam/rectified_buoy': [
                self.create_publisher(Image, '/perception/stereo/debug/left_cam/rectified_buoy1', 10)
            ],
            '/perception/stereo/debug/right_cam/rectified_buoy': [
                self.create_publisher(Image, '/perception/stereo/debug/right_cam/rectified_buoy1', 10)
            ]
        }

        self.single_debug_pubs = {
            '/perception/stereo/debug/points': self.create_publisher(PointCloud2,
                '/perception/stereo/debug/points', 10)
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.get_parameter('debug').value:
            node = self
        else:
            node = None
        self.buoy_stereo = BuoyStereo(node=node,
            multiprocessing=self.get_parameter('multiprocessing').value)
    
    def left_cam_callback(self, msg:Image):
        self.left_image = msg
    
    def right_cam_callback(self, msg:Image):
        self.right_image = msg
    
    def left_cam_info_callback(self, msg:CameraInfo):
        self.left_cam_info = msg
    
    def right_cam_info_callback(self, msg:CameraInfo):
        self.right_cam_info = msg
    
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
    
    def debug_pub(self, name:str, msg):
        self.single_debug_pubs[name].publish(msg)
    
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
        
        self.buoy_stereo.cam_transform = cv2_trans
        
    def execute_buoy_stereo(self):

        if self.buoy_stereo.cam_transform is None:
            self.find_cam_transform()
            return BuoyArray()
        
        self.buoy_stereo.run()

        if self.buoy_stereo.buoys is None:
            return BuoyArray()
        
        self.buoy_stereo.buoys.header.stamp = self.get_clock().now().to_msg()
        self.buoy_stereo.buoys.header.frame_id = self.frames[0]

        return self.buoy_stereo.buoys
    
    def srv_callback(self, req:ImageBuoyStereo.Request, res:ImageBuoyStereo.Response):
        self.get_logger().info('buoy stereo received request')
        if (self.left_image is None or self.right_image is None 
            or self.left_cam_info is None or self.right_cam_info is None):
            self.get_logger().info('Missing images or camera info')
            return res
        
        self.buoy_stereo.left_img_contours = None
        self.buoy_stereo.right_img_contours = None
        
        left_msg = ImageBuoyFilter.Request()
        left_msg.image = self.left_image
        left_msg.camera_info = self.left_cam_info

        right_msg = ImageBuoyFilter.Request()
        right_msg.image = self.right_image
        right_msg.camera_info = self.right_cam_info

        left_req = self.left_cam_client.call_async(left_msg)
        left_req.add_done_callback(self.left_buoy_filter_response) 

        right_req = self.right_cam_client.call_async(right_msg)
        right_req.add_done_callback(self.right_buoy_filter_response)
        
        while (self.buoy_stereo.left_img_contours is None or
            self.buoy_stereo.right_img_contours is None):
            if self.get_parameter('debug').value:
                self.get_logger().info('Waiting for contours')
            time.sleep(0.5)
            
        buoys = self.execute_buoy_stereo()

        res.buoys = buoys
        res.header.frame_id = self.frames[0]
        
        return res

    def left_buoy_filter_response(self, future):
        result = future.result()
        self.buoy_stereo.left_img_contours = result.contours
        self.buoy_stereo.left_cam_info = result.camera_info
    
    def right_buoy_filter_response(self, future):
        result = future.result()
        self.buoy_stereo.right_img_contours = result.contours
        self.buoy_stereo.right_cam_info = result.camera_info


def main(args=None):
    
    rclpy.init(args=args)

    node = BuoyStereoNode()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.spin()

    # rclpy.spin(node)

    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()