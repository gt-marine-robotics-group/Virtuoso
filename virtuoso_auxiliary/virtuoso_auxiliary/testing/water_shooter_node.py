import rclpy
from rclpy.node import Node
from virtuoso_msgs.srv import ShootWater

NUM_SECONDS = 120.0

class TestWaterShooter(Node):

    def __init__(self):
        super().__init__('test_ball_shooter_node')

        self.shooter_client = self.create_client(ShootWater, 'shoot_water')
        self.shooter_req = None
    
        self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        if self.shooter_req is not None:
            return
        
        msg = ShootWater.Request()
        msg.num_seconds = NUM_SECONDS

        self.shooter_req = self.shooter_client.call_async(msg)
        self.shooter_req.add_done_callback(self.shoot_callback)
    
    def shoot_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Result: {result}')


def main(args=None):
    rclpy.init(args=args)

    node = TestWaterShooter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
