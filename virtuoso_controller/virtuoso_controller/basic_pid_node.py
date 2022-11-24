import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import tf_transformations
import numpy
from .basic_pid import BasicPID

class BasicPIDNode(Node):

    def __init__(self):
        super().__init__('controller_basic_PID')
        
        # self.state_estimate = Odometry()
        # self.target_waypoint = Odometry()
        # self.yaw_integral = 0.0
        # self.x_intergral = 0.0
        # self.y_integral = 0.0
        # self.previous_target_waypoint = Odometry()
        # self.received_waypoint = False
        # self.navigate_to_point = False

        self.declare_parameters(namespace='', parameters=[
            ('basic_kp', 1.0),
            ('basic_kd', 1.0),
            ('basic_ki', 1.0),
            ('basic_rotate_kp', 1.0),
            ('basic_rotate_kd', 1.0),
            ('basic_rotate_ki', 1.0)
        ])

        self.pid = BasicPID(kp=self.get_parameter('basic_kp').value, 
            kd=self.get_parameter('basic_kd').value, ki=self.get_parameter('basic_ki').value,
            rotate_kp=self.get_parameter('basic_rotate_kp').value,
            rotate_kd=self.get_parameter('basic_rotate_kd').value,
            rotate_ki=self.get_parameter('basic_rotate_ki').value)
                   
        self.target_force_x_pub = self.create_publisher(Float32, 
            '/controller/basic_pid/targetForceX', 10)
        self.target_force_y_pub = self.create_publisher(Float32,
            '/controller/basic_pid/targetForceY', 10)
        self.target_torque_pub = self.create_publisher(Float32, 
            '/controller/basic_pid/targetTorque', 10)
        
        #subscribe to odometry from localization
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odometry_callback,
            10)   
            
        #subscribe to waypoints
        self.waypoint_subscriber = self.create_subscription(
            Odometry,
            '/waypoint',
            self.waypoint_callback,
            10)     
        self.navigate_to_point_subscriber = self.create_subscription(
            Bool,
            '/controller/navigateToPoint',
            self.navigate_to_point_callback,
            10)       

        self.timer = self.create_timer(0.1, self.run_pid)

    def navigate_to_point_callback(self, msg:Bool):
        self.pid.navigate_to_point = msg.data
        	
    def odometry_callback(self, msg:Odometry):
        self.pid.state_estimate = msg
    
    def waypoint_callback(self, msg:Odometry):
        self.pid.target_waypoint = msg
        self.pid.received_waypoint = True

    def run_pid(self):

        x, y, torque = self.pid.run()

        if x is None:
            return
        
        self.target_force_x_pub.publish(x)
        self.target_force_y_pub.publish(y)
        self.target_torque_pub.publish(torque)

        # target_x = self.target_waypoint.pose.pose.position.x
        # target_y = self.target_waypoint.pose.pose.position.y    
        
        # current_vel_x = self.state_estimate.twist.twist.linear.x
        # current_vel_y = self.state_estimate.twist.twist.linear.y
    	
        # current_x = self.state_estimate.pose.pose.position.x
        # current_y = self.state_estimate.pose.pose.position.y
    	
        # velocity_x = target_x - current_x
        # velocity_y = target_y - current_y

        # target_vel = [velocity_x, velocity_y, 0.0, 0.0]
        
        # q = [self.state_estimate.pose.pose.orientation.x, 
        #     self.state_estimate.pose.pose.orientation.y, 
        #     self.state_estimate.pose.pose.orientation.z, 
        #     self.state_estimate.pose.pose.orientation.w]
        # q_inv = q.copy()
        # q_inv[0] = -q_inv[0]
        # q_inv[1] = -q_inv[1]
        # q_inv[2] = -q_inv[2]

        # target_vel = tf_transformations.quaternion_multiply(q_inv, target_vel)
        # target_vel = tf_transformations.quaternion_multiply(target_vel, q)
        
        # kp_factor = self.get_parameter('basic_kp').value
        # kd_factor = self.get_parameter('basic_kd').value
        # ki_factor = self.get_parameter('basic_ki').value
                                
        # if(self.previous_target_waypoint != self.target_waypoint):
        #      self.x_intergral = 0.0
        #      self.y_integral = 0.0
        # self.x_intergral = self.x_intergral + target_vel[0]*0.01
        # self.y_integral = self.y_integral + target_vel[1]*0.01       

        # target_force_y = ((target_vel[1]*0.15*kp_factor - current_vel_y*0.9*0.7*kd_factor) 
        #     + self.y_integral*0.001*ki_factor)
        # target_force_x = ((target_vel[0]*0.11*kp_factor - current_vel_x*0.333*0.7*kd_factor)
        #     + self.x_intergral*0.001*ki_factor)

        # target_force_x = target_force_x * (5/3) * 4
        # target_force_y = target_force_y * (5/3) * 4
        # if(abs(target_force_y) < 0.2):
        #      target_force_y = target_force_y/abs(target_force_y)*0.2
        # if(abs(target_force_x)<0.2):
        #      target_force_x = target_force_x/abs(target_force_x)*0.2
        
        # heading = [1.0, 0.0, 0.0, 0.0]
        # q_target = [self.target_waypoint.pose.pose.orientation.x, 
        #     self.target_waypoint.pose.pose.orientation.y, 
        #     self.target_waypoint.pose.pose.orientation.z, 
        #     self.target_waypoint.pose.pose.orientation.w]
        
        # q_target_inv = q_target.copy()
        # q_target_inv[0] = -q_target_inv[0]
        # q_target_inv[1] = -q_target_inv[1]
        # q_target_inv[2] = -q_target_inv[2]
        # heading = tf_transformations.quaternion_multiply(q_target, heading)
        # heading = tf_transformations.quaternion_multiply(heading, q_target_inv)
              
        # heading = tf_transformations.quaternion_multiply(q_inv, heading)
        # heading = tf_transformations.quaternion_multiply(heading, q)
        # theta_target_heading = numpy.arctan2(heading[1], heading[0])

        # omega = [self.state_estimate.twist.twist.angular.x, 
        #     self.state_estimate.twist.twist.angular.y, 
        #     self.state_estimate.twist.twist.angular.z, 
        #     0.0]  
        # omega = tf_transformations.quaternion_multiply(q, omega)
        # omega = tf_transformations.quaternion_multiply(omega, q_inv)        
        
        # if(self.previous_target_waypoint != self.target_waypoint):
        #      self.yaw_integral = 0.0
        # self.yaw_integral += theta_target_heading*0.01
        
        # kp_rotate_factor = self.get_parameter('basic_rotate_kp').value
        # kd_rotate_factor = self.get_parameter('basic_rotate_kd').value
        # ki_rotate_factor = self.get_parameter('basic_rotate_ki').value       
        
        # target_torque = (theta_target_heading*0.76*kp_rotate_factor 
        #     - omega[2]*1.2*kd_rotate_factor 
        #     + 0.001*self.yaw_integral*ki_rotate_factor)
        
        # target_x_to_send = Float32(data=target_force_x)
        # target_y_to_send = Float32(data=target_force_y)
        # target_torque_to_send = Float32(data=target_torque)
        
        # if(self.received_waypoint):
        #     self.target_force_x_pub.publish(target_x_to_send)
        #     self.target_force_y_pub.publish(target_y_to_send)
        #     self.target_torque_pub.publish(target_torque_to_send)
        
        # self.previous_target_waypoint = self.target_waypoint

        
def main(args=None):
    rclpy.init(args=args)

    basic_PID = BasicPIDNode()

    rclpy.spin(basic_PID)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    basic_PID.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
