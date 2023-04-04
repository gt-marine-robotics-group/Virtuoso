import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from virtuoso_msgs.action import ShootBalls
from std_msgs.msg import Empty, Float32
import time

class BallShooterNode(Node):

    def __init__(self):
        super().__init__('auxiliary_ball_shooter')

        self.declare_parameters(namespace='', parameters=[
            ('sim', False),
            ('shooter_motor_topic', ''),
            ('num_shots', 0),
            ('loading_motor_topic', ''),
            ('shooter_motor_speed', 0.0),
            ('loading_motor_speed', 0.0),
            ('shooter_motor_spinup_time', 0.0),
            ('single_shot_time', 0.0),
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

        if self.sim:
            self.shoot_sim(goal_handle)
            goal_handle.succeed()
            result = ShootBalls.Result()
            result.success = True
            return result

        loop_iter_count = 0
        load_iter_count = 0
        shoot_iter_count = 0
        shooting = False

        shot_count = 0

        self.shooter_pub.publish(Float32(data=self.get_parameter('shooter_motor_speed').value))

        while True:
            time.sleep(0.01)
            if loop_iter_count/100 < self.get_parameter('shooter_motor_spinup_time').value:
                loop_iter_count += 1
                continue
            
            if shooting:
                shoot_iter_count += 1
                if shoot_iter_count/100 > self.get_parameter('single_shot_time').value:
                    if shot_count == self.get_parameter('num_shots').value: 
                        break
                    shooting = False
                    shoot_iter_count = 0
                    load_iter_count = 0
                    self.loading_pub.publish(Float32(data=0.0)) 
                continue
            
            if shot_count == 0 or load_iter_count/100 > self.get_parameter('between_load_time_gap').value:
                shot_count += 1
                load_iter_count = 0
                shooting = True
                self.loading_pub.publish(Float32(data=self.get_parameter('loading_motor_speed').value))
                msg = ShootBalls.Feedback()
                msg.state = f'Shot ball {shot_count}'
                goal_handle.publish_feedback(msg)
                continue
                
            load_iter_count += 1
        
        self.shooter_pub.publish(Float32())
        self.loading_pub.publish(Float32())

        goal_handle.succeed()

        result = ShootBalls.Result()
        result.success = True
        return result

    def shoot_sim(self, goal_handle):
        loop_iter_count = 0
        shot_count = 0
        while True:
            time.sleep(1.0)
            if loop_iter_count < self.get_parameter('between_load_time_gap').value:
                loop_iter_count += 1
                continue
            if shot_count == self.get_parameter('num_shots').value: 
                break
            shot_count += 1
            loop_iter_count = 0
            self.shooter_pub.publish(Empty())
            msg = ShootBalls.Feedback()
            msg.state = f'Shot ball {shot_count}'
            goal_handle.publish_feedback(msg)


def main(args=None):
    rclpy.init(args=args)

    node = BallShooterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
