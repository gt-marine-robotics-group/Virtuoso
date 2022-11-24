from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations

class VelocityPID:

    def __init__(self, kp, kd, ki):

        self._kp_factor = kp
        self._kd_factor = kd
        self._ki_factor = ki

        self.state_estimate  = Odometry()
        self.target_twist = Twist()
        self.received_cmd_vel = False

        self._yaw_integral = 0.0
        self._x_integral = 0.0
        self._y_integral = 0.0
        self._previous_target_twist = Twist()
    
    def run(self):

        current_vel_x = self.state_estimate.twist.twist.linear.x
        current_vel_y = self.state_estimate.twist.twist.linear.y

        target_vel = [self.target_twist.linear.x, 
            self.target_twist.linear.y , 0.0, 0.0]

                
        q = [self.state_estimate.pose.pose.orientation.x, 
            self.state_estimate.pose.pose.orientation.y, 
            self.state_estimate.pose.pose.orientation.z, 
            self.state_estimate.pose.pose.orientation.w]
        q_inv = q.copy()
        q_inv[0] = -q_inv[0]
        q_inv[1] = -q_inv[1]
        q_inv[2] = -q_inv[2]
                
        if(self._previous_target_twist != self.target_twist):
             self._x_integral = 0.0
             self._y_integral = 0.0
        self._x_integral = self._x_integral + (target_vel[0]- current_vel_x)*0.01
        self._y_integral = self._y_integral + (target_vel[1] - current_vel_y)*0.01  
             
        target_force_y = ((target_vel[1]*1.5*self._kp_factor - current_vel_y*self._kd_factor)*0.5 
            + self._y_integral*0.01*self._ki_factor)
        target_force_x = ((target_vel[0]*1.5*self._kp_factor - current_vel_x*self._kd_factor)*0.5 
            + self._x_integral*0.01*self._ki_factor)
        
        omega = [self.state_estimate.twist.twist.angular.x,
            self.state_estimate.twist.twist.angular.y,
            self.state_estimate.twist.twist.angular.z, 0.0]  
        omega = tf_transformations.quaternion_multiply(q, omega)
        omega = tf_transformations.quaternion_multiply(omega, q_inv)        
        yaw_rate = omega[2]       
        
        target_yaw_rate = self.target_twist.angular.z
        
        if(self._previous_target_twist != self.target_twist):
             self._yaw_integral = 0.0
        self._yaw_integral = self._yaw_integral + (target_yaw_rate - yaw_rate)*0.01
        target_torque = (target_yaw_rate - yaw_rate)*3.0 + 0.01*self._yaw_integral

        target_x_to_send = Float32(data=target_force_x)
        target_y_to_send = Float32(data=target_force_y)
        target_torque_to_send = Float32(data=target_torque)

        if(self.received_cmd_vel):
            return target_x_to_send, target_y_to_send, target_torque_to_send

        self._previous_target_twist = self.target_twist

        return (None, None, None)