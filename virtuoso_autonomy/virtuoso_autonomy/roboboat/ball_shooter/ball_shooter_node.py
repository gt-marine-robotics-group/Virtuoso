import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from .ball_shooter_states import State

class BallShooterNode(Node):

    def __init__(self):
        super().__init__('autonomy_ball_shooter')

        self.waypoint_player_success_sub = self.create_subscription(PoseStamped,
            '/navigation/waypoint_player/success', self.waypoint_player_success_callback, 10)

        self.state = State.NAVIGATING

        self.create_timer(1.0, self.execute)
    
    def waypoint_player_success_callback(self, msg:PoseStamped):
        self.get_logger().info('Waypoint player finished')

        self.state = State.SHOOTING
    
    def execute(self):
        self.get_logger().info(str(self.state))

        if self.state == State.NAVIGATING:
            return
        if self.state == State.SHOOTING:
            self.shoot()
            return
    
    def shoot(self):
        self.get_logger().info('shooting')


def main(args=None):
    rclpy.init(args=args)

    node = BallShooterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()