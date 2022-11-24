import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
import tf_transformations
import numpy

#This node sends the appropriate waypoints to the basic PID, and also decides whether to use the 
#velocity PID for translational movement or the basic PID
class ChoosePID(Node):

    def __init__(self):
        super().__init__('controller_choose_PID')
        
        self.state_estimate = Odometry()
        self.destination = Pose()
        self.navigate_to_point = Bool(data=False)
        self.received_path = False
        self.next_waypoint = Pose()
        self.cmd_vel = Twist()
        self.hold_final_orient = False
        
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
            '/cmd_vel',
            self.cmd_vel_callback,
            10)  
            
        self.hold_final_orientation_sub = self.create_subscription(
            Bool, '/controller/is_translation', self.hold_final_orient_callback, 10)

        self.navigate_to_point_pub = self.create_publisher(Bool, '/controller/navigateToPoint', 10)
        self.waypoint_pub = self.create_publisher(Odometry, '/waypoint', 10)        
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def path_callback(self, msg:Path):
        self.destination = msg.poses[-1].pose 
        self.next_waypoint = msg.poses[0].pose
        self.received_path = True
        
    def cmd_vel_callback(self, msg:Twist):
        self.cmd_vel = msg
        
    def hold_final_orient_callback(self, msg:Bool):
        self.hold_final_orient = msg.data
       
    def timer_callback(self):
        if not self.received_path:
            return

        dest_x = self.destination.position.x
        dest_y = self.destination.position.y

        self_x = self.state_estimate.pose.pose.position.x
        self_y = self.state_estimate.pose.pose.position.y

        distance = ((dest_x - self_x)**2 + (dest_y - self_y)**2)**(1/2)
        #self.get_logger().info('distance: ' + str(distance)) 
        if(distance < 2.0):
            self.navigate_to_point.data = True
        else:
            self.navigate_to_point.data = False
        self.navigate_to_point_pub.publish(self.navigate_to_point)

        target_waypoint = Odometry()
        #If we're within 2 m, point at the final heading. If greater than 2 m,
        #point at the orientation corresponding to the cmd velocity
        if(distance < 2.0 or self.hold_final_orient):
            target_waypoint.pose.pose = self.destination
        else:
            target_waypoint.pose.pose.position = self.destination.position
            theta_cmd_vel = numpy.arctan2(self.cmd_vel.linear.y, self.cmd_vel.linear.x)
            target_orient_body = [0, 0, numpy.sin(theta_cmd_vel/2), numpy.cos(theta_cmd_vel/2)]
            
            q = [self.state_estimate.pose.pose.orientation.x, 
                self.state_estimate.pose.pose.orientation.y, 
                self.state_estimate.pose.pose.orientation.z, 
                self.state_estimate.pose.pose.orientation.w]
            
            target_orient = tf_transformations.quaternion_multiply(q, target_orient_body)
            
            target_quat = Quaternion()
            target_quat.x = target_orient[0]
            target_quat.y = target_orient[1]
            target_quat.z = target_orient[2]
            target_quat.w = target_orient[3]
            
            target_waypoint.pose.pose.orientation = target_quat

        self.waypoint_pub.publish(target_waypoint)
  
    def odometry_callback(self, msg):
        self.state_estimate = msg       

        
def main(args=None):
    rclpy.init(args=args)

    choose_PID = ChoosePID()

    rclpy.spin(choose_PID)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    choose_PID.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
