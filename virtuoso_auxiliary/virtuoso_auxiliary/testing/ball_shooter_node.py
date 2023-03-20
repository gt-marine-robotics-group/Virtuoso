import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from virtuoso_msgs.action import ShootBalls
from rclpy.action import ActionClient

class TestBallShooterNode(Node):

    def __init__(self):
        super().__init__('test_ball_shooter_node')

        self.ball_shooter_client = ActionClient(self, ShootBalls, 'shoot_balls')
        self.ball_shooter_req = None

        self.create_timer(1.0, self.execute)
    
    def execute(self):
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

    node = TestBallShooterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()