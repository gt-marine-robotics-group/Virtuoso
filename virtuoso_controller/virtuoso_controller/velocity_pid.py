from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations

#Outputs target forces to drive vehicle to target cmd_vel

class VelocityPID:

    def __init__(self, k_drag_x, k_error_x, ki_x, k_drag_y, k_error_y, ki_y):

        self._k_drag_factor_x = k_drag_x #Gain on magnitude of target velocity (to combat drag)
        self._k_error_factor_x = k_error_x  #Gain on magnitude of velocity error
        self._ki_factor_x = ki_x  #gain on integral of error
        self._k_drag_factor_y = k_drag_y #Gain on magnitude of target velocity (to combat drag)
        self._k_error_factor_y = k_error_y  #Gain on magnitude of velocity error
        self._ki_factor_y = ki_y  #gain on integral of error
        
        self.state_estimate  = Odometry()
        self.target_twist = Twist() #target twist in base_link frame
        self.received_cmd_vel = False

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
        
        #if new target velocity, reset error integral
        if(self._previous_target_twist != self.target_twist):
             self._x_integral = 0.0
             self._y_integral = 0.0
        #calculate error integral
        self._x_integral = self._x_integral + (target_vel[0]- current_vel_x)*0.01
        self._y_integral = self._y_integral + (target_vel[1] - current_vel_y)*0.01  
        
        #calculate target force x and y. Formula is (roughly):
        #target force = target velocity * drag gain + (error in velocity)*gain on error + (integral of error) * (gain on integral error)
        #Note that x and y are in the base_link frame
        target_force_y = 2.0*(target_vel[1]*.25*self._k_drag_factor_y + (target_vel[1] - current_vel_y)*0.5*self._k_error_factor_y
            + self._y_integral*0.01*self._ki_factor_y)
        target_force_x = (target_vel[0]*0.25*self._k_drag_factor_x + (target_vel[0] - current_vel_x)*0.5*self._k_error_factor_x
            + self._x_integral*0.01*self._ki_factor_x)

        target_x_to_send = Float32(data=target_force_x)
        target_y_to_send = Float32(data=target_force_y)

        if(self.received_cmd_vel):
            return target_x_to_send, target_y_to_send

        self._previous_target_twist = self.target_twist

        return (None, None)
