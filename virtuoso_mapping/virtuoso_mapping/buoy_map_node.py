import rclpy
from rclpy.node import Node
from virtuoso_msgs.msg import BuoyArray
from geometry_msgs.msg import PointStamped
from virtuoso_perception.utils.geometry_msgs import do_transform_point
from collections import deque
from tf2_ros import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time

class BuoyMapNode(Node):

    cam_fov = 0.54
    frames = [
        "wamv/front_left_camera_link_optical",
        "wamv/front_right_camera_link_optical"
    ]

    def __init__(self):
        super().__init__('mapping_buoy_map')

        self.buoys_sub = self.create_subscription(BuoyArray, 
            '/perception/stereo/buoys', self.buoys_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cam_transform = None
        
        self.curr_buoys:BuoyArray = None

        self.prev_buoys = deque(maxlen=10)

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
        
        self.cam_transform = trans

    def buoys_callback(self, msg:BuoyArray):
        self.curr_buoys = msg
        self.execute()
    
    def transform_curr_buoys_to_map_frame(self):

        trans:TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                BuoyMapNode.frames[0],
                Time()
        )

        for buoy in self.curr_buoys.buoys:
            ps = PointStamped(point=buoy.location) 
            trans_ps = do_transform_point(ps, trans)
            buoy.location = trans_ps.point
    
    def execute(self):

        self.get_logger().info(f'original: {self.curr_buoys}') 
        try:
            self.transform_curr_buoys_to_map_frame()
        except Exception:
            return
        self.get_logger().info(f'transformed: {self.curr_buoys}')


def main(args=None):
    
    rclpy.init(args=args)

    node = BuoyMapNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
