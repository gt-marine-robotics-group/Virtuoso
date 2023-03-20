import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from .ball_shooter_states import State
from rclpy.action import ActionClient
from virtuoso_msgs.action import ShootBalls

class BallShooterNode(Node):

    def __init__(self):
        super().__init__('autonomy_ball_shooter')

        self.waypoint_player_success_sub = self.create_subscription(PoseStamped,
            '/navigation/waypoint_player/success', self.waypoint_player_success_callback, 10)

        self.state = State.NAVIGATING

        self.ball_shooter_client = ActionClient(self, ShootBalls, 'shoot_balls')
        self.ball_shooter_req = None
        self.ball_shooter_result = None

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

        if self.ball_shooter_req is not None:
            return

        msg = ShootBalls.Goal()
        
        self.ball_shooter_req = self.ball_shooter_client.send_goal_async(msg,
            feedback_callback=self.ball_shooter_feedback_callback)

        self.ball_shooter_req.add_done_callback(self.ball_shooter_response_callback)
    
    def ball_shooter_feedback_callback(self, msg):
        feedback = msg.feedback
        self.get_logger().info(f'Feedback: {feedback}')
    
    def ball_shooter_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.ball_shooter_result = goal_handle.get_result_async()
        self.ball_shooter_result.add_done_callback(self.ball_shooter_result_callback)
    
    def ball_shooter_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')


def main(args=None):
    rclpy.init(args=args)

    node = BallShooterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()