import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from .choose_pid import ChoosePID

#This node sends the appropriate waypoints to the basic PID, and also decides whether to use the 
#velocity PID for translational movement or the basic PID
class ChoosePIDNode(Node):

    def __init__(self):
        super().__init__('controller_choose_PID')

        self.choose_pid = ChoosePID()
        
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
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/controller/cmd_vel',
            self.cmd_vel_callback,
            10)  
            
        self.hold_final_orientation_sub = self.create_subscription(
            Bool, '/controller/is_translation', self.hold_final_orient_callback, 10)

        self.navigate_to_point_pub = self.create_publisher(Bool, '/controller/navigateToPoint', 10)
        self.waypoint_pub = self.create_publisher(Odometry, '/controller/waypoint', 10)        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def path_callback(self, msg:Path):
        self.choose_pid.destination = msg.poses[-1].pose
        self.choose_pid.next_waypoint = msg.poses[0].pose
        self.choose_pid.received_path = True
        
    def cmd_vel_callback(self, msg:Twist):
        self.choose_pid.cmd_vel = msg
        
    def hold_final_orient_callback(self, msg:Bool):
        self.choose_pid.hold_final_orient = msg.data
       
    def timer_callback(self):
        nav_to_point, target_waypoint = self.choose_pid.run()

        if nav_to_point is None:
            return

        self.navigate_to_point_pub.publish(nav_to_point)
        self.waypoint_pub.publish(target_waypoint)
  
    def odometry_callback(self, msg):
        self.choose_pid.state_estimate = msg

        
def main(args=None):
    rclpy.init(args=args)

    choose_PID = ChoosePIDNode()

    rclpy.spin(choose_PID)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    choose_PID.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
