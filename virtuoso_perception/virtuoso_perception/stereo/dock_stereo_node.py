#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from virtuoso_perception.stereo.dock_stereo import DockStereo
from virtuoso_msgs.srv import ImageDockStereo, ImageBuoyFilter
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from virtuoso_perception.stereo.utils import tf_transform_to_cv2_transform
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

class DockStereoNode(Node):

    def __init__(self):
        super().__init__('perception_dock_stereo')

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

        # self.srv = self.create_service(ImageBuoyStereo, 'perception/image_buoy_stereo',
        #     self.srv_callback, callback_group=self.cb_group_1)
        self.srv = self.create_service(ImageDockStereo, 'perception/dock_stereo',
            self.srv_callback, callback_group=self.cb_group_1)
        
        self.left_cam_client = self.create_client(ImageBuoyFilter, 
            f'{cams[0]}/find_dock_posts', callback_group=self.cb_group_2)
        self.right_cam_client = self.create_client(ImageBuoyFilter,
            f'{cams[1]}/find_dock_posts', callback_group=self.cb_group_3)

        self.single_debug_pubs = {
            '/perception/stereo/debug/points': self.create_publisher(PointCloud2,
                '/perception/stereo/debug/points', 10),
            '/perception/dock_stereo/left_cam_left': self.create_publisher(Image,
                '/perception/dock_stereo/left_cam_left', 10),
            '/perception/dock_stereo/left_cam_right': self.create_publisher(Image,
                '/perception/dock_stereo/left_cam_right', 10),
            '/perception/dock_stereo/right_cam_left': self.create_publisher(Image,
                '/perception/dock_stereo/right_cam_left', 10),
            '/perception/dock_stereo/right_cam_right': self.create_publisher(Image,
                '/perception/dock_stereo/right_cam_right', 10)
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.get_parameter('debug').value:
            node = self
        else:
            node = None
        self.dock_stereo = DockStereo(node=node,
            multiprocessing=self.get_parameter('multiprocessing').value)
    
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
        
        self.dock_stereo.cam_transform = cv2_trans
        
    def execute_dock_stereo(self):

        if self.dock_stereo.cam_transform is None:
            self.find_cam_transform()
            return list()
        
        try:
            self.dock_stereo.run()
        except Exception as e:
            self.get_logger().info(str(e))

        if self.dock_stereo.end_points is None:
            return list()
        
        return self.dock_stereo.end_points
    
    def srv_callback(self, req:ImageDockStereo.Request, res:ImageDockStereo.Response):
        self.get_logger().info('received req')
        self.dock_stereo.left_img_contours = None
        self.dock_stereo.right_img_contours = None
        
        left_msg = ImageBuoyFilter.Request()

        right_msg = ImageBuoyFilter.Request()

        left_req = self.left_cam_client.call_async(left_msg)
        left_req.add_done_callback(self.left_code_filter_response) 

        right_req = self.right_cam_client.call_async(right_msg)
        right_req.add_done_callback(self.right_code_filter_response)
        
        while (self.dock_stereo.left_img_contours is None or
            self.dock_stereo.right_img_contours is None):
            if self.get_parameter('debug').value:
                self.get_logger().info('Waiting for contours')
            time.sleep(0.5)
            
        end_points = self.execute_dock_stereo()

        res.end_points = end_points
        res.header.frame_id = self.frames[0]
        
        return res

    def left_code_filter_response(self, future):
        result = future.result()
        self.dock_stereo.left_img_contours = result.contours
        self.dock_stereo.left_cam_info = result.camera_info
    
    def right_code_filter_response(self, future):
        result = future.result()
        self.dock_stereo.right_img_contours = result.contours
        self.dock_stereo.right_cam_info = result.camera_info


def main(args=None):
    
    rclpy.init(args=args)

    node = DockStereoNode()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.spin()

    # rclpy.spin(node)

    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()