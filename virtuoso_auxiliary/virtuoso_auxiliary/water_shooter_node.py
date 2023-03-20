import rclpy
from rclpy.node import Node
from virtuoso_msgs.srv import ShootWater
from std_msgs.msg import Float32
import time

class WaterShooterNode(Node):

    def __init__(self):
        super().__init__('auxiliary_water_shooter')

        self.srv = self.create_service(ShootWater, 'shoot_water',
            self.srv_callback)
        
        self.shooter_pub = self.create_publisher(Float32, '/water_shooter/throttle_cmd', 10)
    
    def srv_callback(self, req:ShootWater.Request, res:ShootWater.Response):
        self.get_logger().info('request received')

        self.shooter_pub.publish(Float32(data=1.0))

        time.sleep(req.num_seconds)
        
        self.shooter_pub.publish(Float32())

        res.success = True
        return res


def main(args=None):
    rclpy.init(args=args)

    node = WaterShooterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()