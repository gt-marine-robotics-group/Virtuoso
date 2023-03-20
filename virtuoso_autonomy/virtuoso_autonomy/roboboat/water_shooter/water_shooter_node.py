import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from virtuoso_msgs.srv import ShootWater
from .water_shooter_states import State
import time

class WaterShooterNode(Node):

    def __init__(self):
        super().__init__('autonomy_water_shooter')

        self.declare_parameters(namespace='', parameters=[
            ('shoot_seconds', 0.0)
        ])

        self.waypoint_player_success_sub = self.create_subscription(PoseStamped,
            '/navigation/waypoint_player/success', self.waypoint_player_success_callback, 10)
        
        self.shooter_client = self.create_client(ShootWater, 'shoot_water')
        self.shooter_req = None

        self.state = State.NAVIGATING

        self.create_timer(1.0, self.execute)
    
    def waypoint_player_success_callback(self, msg:PoseStamped):
        self.get_logger().info('Waypoint player finished')

        self.state = State.SHOOTING

        time.sleep(3.0)  
    
    def execute(self):
        self.get_logger().info(str(self.state))

        if self.state == State.NAVIGATING:
            return
        if self.state == State.SHOOTING:
            self.shoot()
            return
    
    def shoot(self):
        self.get_logger().info('shooting')

        if self.shooter_req is not None:
            return
        
        msg = ShootWater.Request()
        msg.num_seconds = self.get_parameter('shoot_seconds').value

        self.shooter_req = self.shooter_client.call_async(msg)
        self.shooter_req.add_done_callback(self.shoot_callback)
    
    def shoot_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Result: {result}')
        self.state = State.COMPLETE
        

def main(args=None):
    rclpy.init(args=args)

    node = WaterShooterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()