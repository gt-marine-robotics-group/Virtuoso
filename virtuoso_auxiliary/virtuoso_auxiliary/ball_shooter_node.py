import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from virtuoso_msgs.action import ShootBalls
from std_msgs.msg import Empty, Float32

class BallShooterNode(Node):

    def __init__(self):
        super().__init__('auxiliary_ball_shooter')

        self.declare_parameters(namespace='', parameters=[
            ('sim', False),
            ('shooter_motor_topic', ''),
            ('loading_motor_topic', ''),
            ('shooter_motor_speed', 0.0),
            ('loading_motor_speed', 0.0),
            ('shooter_motor_spinup_time', 0.0),
            ('single_load_time', 0.0),
            ('between_load_time_gap', 0.0)
        ])

        self.sim = self.get_parameter('sim').value

        self.action_server = ActionServer(self, ShootBalls, 'shoot_balls', 
            self.action_server_callback)
        
        if self.sim:
            self.shooter_pub = self.create_publisher(Empty, 
                self.get_parameter('shooter_motor_topic').value, 10)
        else:
            self.shooter_pub = self.create_publisher(Float32,
                self.get_parameter('shooter_motor_topic').value, 10)
            self.loading_pub = self.create_publisher(Float32,
                self.get_parameter('loading_motor_topic').value, 10)

    def action_server_callback(self, goal_handle):
        self.get_logger().info('Received action request')

        result = ShootBalls.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)

    node = BallShooterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()