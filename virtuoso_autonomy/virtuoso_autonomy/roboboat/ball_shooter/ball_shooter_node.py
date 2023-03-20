import rclpy
from rclpy.node import Node

class BallShooterNode(Node):

    def __init__(self):
        super().__init__('autonomy_ball_shooter')


def main(args=None):
    rclpy.init(args=args)

    node = BallShooterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()