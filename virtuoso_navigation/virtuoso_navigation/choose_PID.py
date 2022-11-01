import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
#This node sends the appropriate waypoints to the basic PID, and also decides whether to use the 
#velocity PID for translational movement or the basic PID
import tf_transformations

import numpy

class choosePID(Node):

    def __init__(self):
        super().__init__('choose_PID')
        
        self.stateEstimate = Odometry()
        self.destination = Pose()
        self.navigateToPoint = Bool()
        self.navigateToPoint.data = False
        self.receivedPath = False
        self.nextWaypoint = Pose()
        self.cmd_vel = Twist()
        self.hold_final_orient = False
        
        self.path_subscriber = self.create_subscription(
            Path,
            '/transformed_global_plan',
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

        self.navigateToPointPub = self.create_publisher(Bool, '/navigation/navigateToPoint', 10)
        self.waypointPub = self.create_publisher(Odometry, '/waypoint', 10)        
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def path_callback(self, msg):
        self.destination = msg.poses[-1].pose 
        self.nextWaypoint = msg.poses[0].pose
        self.receivedPath = True
        
    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        
    def hold_final_orient_callback(self, msg):
        self.hold_final_orient = msg.data
       
    def timer_callback(self):
        if(self.receivedPath):
             destX = self.destination.position.x
             destY = self.destination.position.y
        
             selfX = self.stateEstimate.pose.pose.position.x
             selfY = self.stateEstimate.pose.pose.position.y
        
             distance = ((destX - selfX)**2 + (destY - selfY)**2)**(1/2)
             #self.get_logger().info('distance: ' + str(distance)) 
             if(distance < 2.0):
                  self.navigateToPoint.data = True
             else:
                  self.navigateToPoint.data = False
             self.navigateToPointPub.publish(self.navigateToPoint)
        
             targetWaypoint = Odometry()
             #If we're within 2 m, point at the final heading. If greater than 2 m,
             #point at the orientation corresponding to the cmd velocity
             if(distance < 2.0 or self.hold_final_orient):
                  targetWaypoint.pose.pose = self.destination
             else:
     	          targetWaypoint.pose.pose.position = self.destination.position
     	          theta_cmd_vel = numpy.arctan2(self.cmd_vel.linear.y, self.cmd_vel.linear.x)
     	          target_orient_body = [0, 0, numpy.sin(theta_cmd_vel/2), numpy.cos(theta_cmd_vel/2)]
     	          
     	          q = [self.stateEstimate.pose.pose.orientation.x, self.stateEstimate.pose.pose.orientation.y, self.stateEstimate.pose.pose.orientation.z, self.stateEstimate.pose.pose.orientation.w]
     	          
     	          target_orient = tf_transformations.quaternion_multiply(q, target_orient_body)
     	          
     	          target_quat = Quaternion()
     	          target_quat.x = target_orient[0]
     	          target_quat.y = target_orient[1]
     	          target_quat.z = target_orient[2]
     	          target_quat.w = target_orient[3]
     	           
     	          targetWaypoint.pose.pose.orientation = target_quat
     	          #targetWaypoint.pose.pose.orientation = self.nextWaypoint.orientation
             self.waypointPub.publish(targetWaypoint)
  
    def odometry_callback(self, msg):
        self.stateEstimate = msg       

        
def main(args=None):
    rclpy.init(args=args)

    choose_PID = choosePID()

    rclpy.spin(choose_PID)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    choose_PID.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
