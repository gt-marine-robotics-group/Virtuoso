from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist, Quaternion
from nav_msgs.msg import Odometry
import tf_transformations
import numpy

class ChoosePID:

    def __init__(self):

        self.state_estimate = Odometry()
        self.destination = Pose()
        self.navigate_to_point = Bool(data=False)
        self.received_path = False
        self.next_waypoint = Pose()
        self.cmd_vel = Twist()
        self.hold_final_orient = False
    
    def run(self):
        if not self.received_path:
            return None, None

        dest_x = self.destination.position.x
        dest_y = self.destination.position.y

        self_x = self.state_estimate.pose.pose.position.x
        self_y = self.state_estimate.pose.pose.position.y

        distance = ((dest_x - self_x)**2 + (dest_y - self_y)**2)**(1/2)
        if(distance < 2.0):
            self.navigate_to_point.data = True
        else:
            self.navigate_to_point.data = False

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

        return self.navigate_to_point, target_waypoint