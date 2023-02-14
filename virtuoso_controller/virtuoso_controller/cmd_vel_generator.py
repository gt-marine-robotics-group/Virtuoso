from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry, Path
import tf_transformations
import numpy

#outputs a cmd_vel in the base_link frame to follow a path

class CmdVelGenerator:

    def __init__(self):

        self.state_estimate = Odometry()
        self.destination = Pose()
        self.received_path = False
        self.next_waypoint = Pose()
        self.second_waypoint = Pose()
        self.nav2_path = Path()
        self.hold_final_orient = False
    
    def run(self):

        if not self.received_path:
            return None

        dest_x = self.destination.position.x
        dest_y = self.destination.position.y

        self_x = self.state_estimate.pose.pose.position.x
        self_y = self.state_estimate.pose.pose.position.y
        
        dist_to_target = ((self_x - dest_x)**2 + (self_y - dest_y)**2)**(1/2)
        
        min_pose_distance = 100000.0
        closest_pose = 0
        
        #pick out the closest pose and the pose after that pose
        for i in range(0,len(self.nav2_path.poses)):

            next_pose_distance = (
                (self.nav2_path.poses[i].pose.position.x - self_x)**2 + 
                (self.nav2_path.poses[i].pose.position.y - self_y)**2
            )**(1/2)

            if (next_pose_distance < min_pose_distance):
                closest_pose = i
                min_pose_distance = next_pose_distance
                        
        next_x = self.nav2_path.poses[closest_pose].pose.position.x
        next_y = self.nav2_path.poses[closest_pose].pose.position.y
        #get the pose after the closest pose. If the closest pose is the last pose, then just copy the closest pose
        if (len(self.nav2_path.poses) > closest_pose + 1):
            second_x = self.nav2_path.poses[closest_pose + 1].pose.position.x
            second_y = self.nav2_path.poses[closest_pose + 1].pose.position.y
        else:
            second_x = next_x
            second_y = next_y
        
        #Get the minimum pose distance as the distance between the current position and the line
        #created by the closest pose and the next pose after that
        #This formula obviously only works if the closest and second poses aren't the same point
        if (second_x != next_x or second_y != next_y):
            min_pose_distance = abs(
                (second_x - next_x)*(next_y - self_y) - (next_x - self_x)*(second_y - next_y)
            ) / ((second_x - next_x)**2 + (second_y - next_y)**2)**(1/2)    

        q = [self.state_estimate.pose.pose.orientation.x, 
            self.state_estimate.pose.pose.orientation.y, 
            self.state_estimate.pose.pose.orientation.z, 
            self.state_estimate.pose.pose.orientation.w]
        q_inv = q.copy()
        q_inv[0] = -q_inv[0]
        q_inv[1] = -q_inv[1]
        q_inv[2] = -q_inv[2]  
        
        #velocity parallel to the path
        vel_parallel = [float(second_x - next_x), float(second_y - next_y), 0.0, 0.0]
        
        #transform the velocity parallel to the path to the base_link frame
        vel_parallel = tf_transformations.quaternion_multiply(q_inv, vel_parallel)
        vel_parallel2 = tf_transformations.quaternion_multiply(vel_parallel, q)      
        vel_parallel = [vel_parallel2[0], vel_parallel2[1], vel_parallel2[2], vel_parallel2[3]]       
        
        #angle between the vehicle's +x and the target velocity
        vel_angle = numpy.arctan2(vel_parallel[0], vel_parallel[1])
        
        vel_parallel_speed = 2.0
        #slow down if we're getting close to the target
        if (dist_to_target < 6.0):
            vel_parallel_speed = dist_to_target/3.0
        #enforce a minimimum velocity though
        if (vel_parallel_speed < 0.3):
            vel_parallel_speed = 0.3
        
        #If we're far away from the path, reduce the parallel velocity
        vel_parallel_speed = vel_parallel_speed - vel_parallel_speed*min_pose_distance/3.0
        
        if (vel_parallel_speed < 0.3):
            vel_parallel_speed = 0.3
        
        
        vel_parallel_mag = vel_parallel_speed/((vel_parallel[0])**2 + (vel_parallel[1])**2)**(1/2)
        
        #now actually apply the calculated magnitude of the parallel velocity to its components
        vel_parallel[0] = vel_parallel[0]*vel_parallel_mag
        vel_parallel[1] = vel_parallel[1]*vel_parallel_mag
        
        if (numpy.isnan(vel_parallel[0])):
            vel_parallel[0] = 0.0
        if (numpy.isnan(vel_parallel[1])):
            vel_parallel[1] = 0.0
        
        #calculate the point on the path that is closest to the current position
        #In the case that the closest and second closest poses aren't the same point, then
        #this closest point will likely lie on the line between them. Imagine a line perpendicular to the
        #path intersecting with the current position. The intersection of this line with the path
        #will be the closest point (not to be confused with the closest pose in the array of poses)
        closest_x = 0.0
        closest_y = 0.0
        
        if (second_x != next_x and second_y != next_y):
            m1 = (second_y - next_y)/(second_x - next_x)
            m2 = -1/m1
            closest_x = (m2*self_x - m1*next_x + next_y - self_y)/(m2 - m1)
            closest_y = m2*(closest_x - self_x) + self_y
        #special cases if the poses happen to be perfectly aligned in the x or y axes
        elif (second_x != next_x and second_y == next_y):
            closest_y = second_y
            closest_x = self_x
        elif (second_x == next_x and second_y != next_x):
            closest_x = second_x
            closest_y = self_y
        #if the closest pose and the second closest pose are the same point, then the closest point is just that point
        else:
            closest_x = next_x
            closest_y = next_y
        
        #velocity in the map frame towards the path
        vel_towards = [float(closest_x - self_x), float(closest_y - self_y), 0.0, 0.0]
        #transform this velocity to the base_link frame
        vel_towards = tf_transformations.quaternion_multiply(q_inv, vel_towards)
        vel_towards2 = tf_transformations.quaternion_multiply(vel_towards, q)      
        vel_towards = [vel_towards2[0], vel_towards2[1], vel_towards2[2], vel_towards2[3]]       
        
        #the closer to the path, the smaller the speed towards
        speed_towards = min_pose_distance/2.0
        #enforce maximum and minimum speeds towards
        if (speed_towards > 2.0):
            speed_towards = 2.0
        if (speed_towards < 0.05):
            speed_towards = 0.05
        
        if (speed_towards < 0.05):
            speed_towards = 0.05
                    
        vel_towards_mag = speed_towards/((vel_towards[0])**2 + (vel_towards[1])**2)**(1/2)
        
        #apply the speed to the velocity components
        vel_towards[0] = vel_towards[0]*vel_towards_mag
        vel_towards[1] = vel_towards[1]*vel_towards_mag      
        
        #calculate vector sum of parallel and perpendicular velocities
        vel_to_command = Twist()
        vel_to_command.linear.x = vel_parallel[0] + vel_towards[0]
        vel_to_command.linear.y = vel_parallel[1] + vel_towards[1]
        
        #calculate angle between the current vehicle +x axis and the target velocity
        vel_angle = numpy.arctan2(vel_to_command.linear.x, vel_to_command.linear.y)
        
        #if we're trying to point at the target velocity, we want to reduce the magnitude of the target velocity
        #so the vehicle has time and control authority to turn towards the target velocity
        if (not self.hold_final_orient):
            vel_to_command.linear.x = (vel_to_command.linear.x 
                - vel_to_command.linear.x*abs(vel_angle)/numpy.pi/2)
            vel_to_command.linear.y = (vel_to_command.linear.y 
                - vel_to_command.linear.y*abs(vel_angle)/numpy.pi/2)
        
        return vel_to_command
