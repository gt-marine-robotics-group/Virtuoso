import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from .cmd_vel_generator import CmdVelGenerator
import numpy

#outputs a cmd_vel in the base_link frame to follow a path

class CmdVelGeneratorNode(Node):

    def __init__(self):
        super().__init__('controller_cmd_vel_generator')

        self.generator = CmdVelGenerator()
        self.generator.node = self
        
        self.old_path = Path()
        
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
        #If is_translation is true, the vehicle will hold the final orientation instead of turning towards the cmd_vel
        self.hold_final_orientation_sub = self.create_subscription(
            Bool, '/controller/is_translation', self.hold_final_orient_callback, 10) 
            
        self.cmd_vel_publisher = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.path_complete_publisher = self.create_publisher(Bool, '/controller/path_complete', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        
    def hold_final_orient_callback(self, msg:Bool):
        self.generator.hold_final_orient = msg.data
        
    def path_callback(self, msg:Path):
        
        if(self.old_path == msg):
            return
        
        self.generator.destination = msg.poses[-1].pose
        self.generator.nav2_path = msg
        self.generator.received_path = True
        self.generator.completedPoses = numpy.zeros(len(msg.poses))
        self.old_path = msg

       
    def timer_callback(self):
        cmd = self.generator.run()
        if cmd is None:
            return
        pathComplete = Bool()       
        pathComplete.data = False 
        if(self.generator.completedPoses[-1] == 2):
            pathComplete.data = True
        self.path_complete_publisher.publish(pathComplete)
        
        self.cmd_vel_publisher.publish(cmd)
  
    def odometry_callback(self, msg:Odometry):
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
