import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf_transformations


class VelocityPID(Node):

    def __init__(self):
        super().__init__('controller_velocity_PID')

        self.declare_parameter('velocity_kp', 1.0)
        self.declare_parameter('velocity_kd', 1.0)
        self.declare_parameter('velocity_ki', 1.0)
        
        self.state_estimate  = Odometry()
        self.target_twist = Twist()
        self.yaw_integral = 0.0
        self.x_integral = 0.0
        self.y_integral = 0.0
        self.previous_target_twist = Twist()
        self.received_cmd_vel = False

        self.target_force_x_pub = self.create_publisher(Float32, 
            '/controller/velocity_pid/targetForceX', 10)
        self.target_force_y_pub = self.create_publisher(Float32, 
            '/controller/velocity_pid/targetForceY', 10)
        self.target_torque_pub = self.create_publisher(Float32, 
            '/controller/velocity_pid/targetTorque', 10)
        
        #subscribe to odometry from localization
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odometry_callback,
            10)   
            
        #subscribe to command velocity
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)    

        self.timer2 = self.create_timer(0.01, self.run_pid)
        
    def odometry_callback(self, msg:Odometry):
        self.state_estimate = msg
    
    def cmd_vel_callback(self, msg:Twist):
        self.target_twist = msg    
        self.received_cmd_vel = True

    def run_pid(self): 
        
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
                
        if(self.previous_target_twist != self.target_twist):
             self.x_integral = 0.0
             self.y_integral = 0.0
        self.x_integral = self.x_integral + (target_vel[0]- current_vel_x)*0.01
        self.y_integral = self.y_integral + (target_vel[1] - current_vel_y)*0.01  

        kp_factor = self.get_parameter('velocity_kp').value
        kd_factor = self.get_parameter('velocity_kd').value
        ki_factor = self.get_parameter('velocity_ki').value
             
        target_force_y = (target_vel[1]*1.5*kp_factor - current_vel_y*kd_factor)*0.5 + self.y_integral*0.01*ki_factor 
        target_force_x = (target_vel[0]*1.5*kp_factor - current_vel_x*kd_factor)*0.5 + self.x_integral*0.01*ki_factor
        
        omega = [self.state_estimate.twist.twist.angular.x,
            self.state_estimate.twist.twist.angular.y,
            self.state_estimate.twist.twist.angular.z, 0.0]  
        omega = tf_transformations.quaternion_multiply(q, omega)
        omega = tf_transformations.quaternion_multiply(omega, q_inv)        
        yaw_rate = omega[2]       
        
        target_yaw_rate = self.target_twist.angular.z
        
        if(self.previous_target_twist != self.target_twist):
             self.yaw_integral = 0.0
        self.yaw_integral = self.yaw_integral + (target_yaw_rate - yaw_rate)*0.01
        targetTorque = (target_yaw_rate - yaw_rate)*3.0 + 0.01*self.yaw_integral
        
        targetXToSend = Float32()
        targetYToSend = Float32()
        targetTorqueToSend = Float32()
        
        targetXToSend.data = target_force_x
        targetYToSend.data = target_force_y
        targetTorqueToSend.data = targetTorque

        if(self.received_cmd_vel):
             self.target_force_x_pub.publish(targetXToSend)
             self.target_force_y_pub.publish(targetYToSend)
             self.target_torque_pub.publish(targetTorqueToSend)

        self.previous_target_twist = self.target_twist

        
def main(args=None):
    rclpy.init(args=args)

    velocity_PID = VelocityPID()

    rclpy.spin(velocity_PID)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_PID.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
