from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import tf_transformations
import numpy

#outputs target force and torque commands to drive vehicle to target waypoint position and orientation

class BasicPID:

    def __init__(self, kp, kd, ki, rotate_kp, rotate_kd, rotate_ki):

        self._kp_factor:float = kp
        self._kd_factor:float = kd
        self._ki_factor:float = ki
        self._kp_rotate_factor:float = rotate_kp
        self._kd_rotate_factor:float = rotate_kd
        self._ki_rotate_factor:float = rotate_ki

        self.state_estimate = Odometry()
        self.target_waypoint = Odometry()
        self.yaw_integral = 0.0
        self.x_intergral = 0.0
        self.y_integral = 0.0
        self.previous_target_waypoint = Odometry()
        self.received_waypoint = False
        self.navigate_to_point = False
    
    def run(self):
        target_x = self.target_waypoint.pose.pose.position.x
        target_y = self.target_waypoint.pose.pose.position.y    
        
        current_vel_x = self.state_estimate.twist.twist.linear.x
        current_vel_y = self.state_estimate.twist.twist.linear.y
    	
        current_x = self.state_estimate.pose.pose.position.x
        current_y = self.state_estimate.pose.pose.position.y
    	
        velocity_x = target_x - current_x
        velocity_y = target_y - current_y
        
        #Note that this is actually the position error
        target_vel = [velocity_x, velocity_y, 0.0, 0.0]
        
        q = [self.state_estimate.pose.pose.orientation.x, 
            self.state_estimate.pose.pose.orientation.y, 
            self.state_estimate.pose.pose.orientation.z, 
            self.state_estimate.pose.pose.orientation.w]
        q_inv = q.copy()
        q_inv[0] = -q_inv[0]
        q_inv[1] = -q_inv[1]
        q_inv[2] = -q_inv[2]
        
        #transform the position error to the base_link frame from the map frame
        target_vel = tf_transformations.quaternion_multiply(q_inv, target_vel)
        target_vel = tf_transformations.quaternion_multiply(target_vel, q)
                                
        if(self.previous_target_waypoint != self.target_waypoint):
             self.x_intergral = 0.0
             self.y_integral = 0.0
        self.x_intergral = self.x_intergral + target_vel[0]*0.01
        self.y_integral = self.y_integral + target_vel[1]*0.01       
        
        #calculate target forces in the base_link frame
        #note that target_vel is the position error 
        target_force_y = 1.0*((target_vel[1]*0.15*self._kp_factor - current_vel_y*0.9*0.7*self._kd_factor) 
            + self.y_integral*0.001*self._ki_factor)
        target_force_x = ((target_vel[0]*0.11*self._kp_factor - current_vel_x*0.333*0.7*self._kd_factor)
            + self.x_intergral*0.001*self._ki_factor)
        
        #arbitrary gain reduction
        target_force_x = target_force_x * (5/3) * 4
        target_force_y = target_force_y * (5/3) * 4
        #minimum throttle below which the motors will not spin
        if(abs(target_force_y) < 0.2):
             target_force_y = target_force_y/abs(target_force_y)*0.2
        if(abs(target_force_x)<0.2):
             target_force_x = target_force_x/abs(target_force_x)*0.2
        
        heading = [1.0, 0.0, 0.0, 0.0]
        q_target = [self.target_waypoint.pose.pose.orientation.x, 
            self.target_waypoint.pose.pose.orientation.y, 
            self.target_waypoint.pose.pose.orientation.z, 
            self.target_waypoint.pose.pose.orientation.w]
        
        q_target_inv = q_target.copy()
        q_target_inv[0] = -q_target_inv[0]
        q_target_inv[1] = -q_target_inv[1]
        q_target_inv[2] = -q_target_inv[2]
        
        #heading is now the vector representing the direction the front of the boat should be pointing
        #in the map frame
        heading = tf_transformations.quaternion_multiply(q_target, heading)
        heading = tf_transformations.quaternion_multiply(heading, q_target_inv)
        
        #transform heading from the map frame to the base_link frame
        heading = tf_transformations.quaternion_multiply(q_inv, heading)
        heading = tf_transformations.quaternion_multiply(heading, q)
        #get the angle error between current heading and target heading
        theta_target_heading = numpy.arctan2(heading[1], heading[0])

        omega = [self.state_estimate.twist.twist.angular.x, 
            self.state_estimate.twist.twist.angular.y, 
            self.state_estimate.twist.twist.angular.z, 
            0.0]  
        #transform omega from the base_link frame to the map frame so that
        #the pure z axis angular velocity can be obtained
        omega = tf_transformations.quaternion_multiply(q, omega)
        omega = tf_transformations.quaternion_multiply(omega, q_inv)        
        
        #integral calcs
        if(self.previous_target_waypoint != self.target_waypoint):
             self.yaw_integral = 0.0
        self.yaw_integral += theta_target_heading*0.01
        
        #calculate target torque from heading error, angular velocity, and integral error
        target_torque = (theta_target_heading*0.76*self._kp_rotate_factor 
            - omega[2]*1.2*self._kd_rotate_factor 
            + 0.001*self.yaw_integral*self._ki_rotate_factor)
        
        target_x_to_send = Float32(data=target_force_x)
        target_y_to_send = Float32(data=target_force_y)
        target_torque_to_send = Float32(data=target_torque)

        self.previous_target_waypoint = self.target_waypoint
        
        if(self.received_waypoint):
            return target_x_to_send, target_y_to_send, target_torque_to_send
        
        return None, None, None
        
