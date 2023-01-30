import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from std_msgs.msg import Bool
from .buoy_lidar import FindBuoys
from virtuoso_msgs.srv import LidarBuoy

class BuoyLidarNode(Node):

    def __init__(self):
        super().__init__('perception_buoy_lidar')

        self.boxes_sub = self.create_subscription(BoundingBoxArray, 'lidar_bounding_boxes', 
            self.lidar_bounding_boxes_callback, 10)

        self.declare_parameters(namespace='', parameters=[
            ('buoy_max_side_length', 0.0),
            ('tall_buoy_min_z', 0.0),
            ('buoy_loc_noise', 0.0),
        ])

        self.find_buoys = FindBuoys(
            buoy_max_side_length=self.get_parameter('buoy_max_side_length').value,
            tall_buoy_min_z=self.get_parameter('tall_buoy_min_z').value,
            buoy_loc_noise=self.get_parameter('buoy_loc_noise').value
        )

        self.srv = self.create_service(LidarBuoy, 'perception/lidar_buoy',
            self.srv_callback)

        self.create_timer(0.1, self.execute)

    def execute(self):
        buoys = self.find_buoys.find_buoys()
        return buoys

    def lidar_bounding_boxes_callback(self, msg:BoundingBoxArray):
        self.find_buoys.lidar_bounding_boxes = msg
    
    def srv_callback(self, req:LidarBuoy.Request, res:LidarBuoy.Response):
        res.buoys = self.execute()
        self.get_logger().info('lidar buoy returning')
        return res
    

def main(args=None):
    
    rclpy.init(args=args)

    node = BuoyLidarNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
