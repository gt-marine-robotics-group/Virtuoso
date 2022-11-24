import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from .cmd_vel_generator import CmdVelGenerator

class CmdVelGeneratorNode(Node):

    def __init__(self):
        super().__init__('controller_cmd_vel_generator')

        self.generator = CmdVelGenerator()
        
        self.path_subscriber = self.create_subscription(
            Path,
            '/navigation/plan',
            self.path_callback,
            10)  
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odometry_callback,
            10)   
        self.hold_final_orientation_sub = self.create_subscription(
            Bool, '/controller/is_translation', self.hold_final_orient_callback, 10)
            
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def hold_final_orient_callback(self, msg:Bool):
        self.generator.hold_final_orient = msg.data
        
    def path_callback(self, msg:Path):
        self.generator.destination = msg.poses[-1].pose
        self.generator.nav2_path = msg
        self.generator.received_path = True
       
    def timer_callback(self):
        cmd = self.generator.run()
        if cmd is None:
            return
        
        self.cmd_vel_publisher.publish(cmd)
  
    def odometry_callback(self, msg:Odometry):
        # self.state_estimate = msg       
        self.generator.state_estimate = msg

        
def main(args=None):
    rclpy.init(args=args)

    cmd_vel_generator = CmdVelGeneratorNode()

    rclpy.spin(cmd_vel_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_vel_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
