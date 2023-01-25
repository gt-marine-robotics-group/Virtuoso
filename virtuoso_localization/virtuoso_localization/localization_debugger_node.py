import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Bool

from tf2_ros.buffer import Buffer

from tf2_ros.transform_listener import TransformListener 
from tf2_ros import TransformBroadcaster

import tf_transformations
import numpy

class LocalizationDebuggerNode(Node):

    def __init__(self):
        super().__init__('localization_debugger')
        self.measured_odom = Odometry()
        self.basic_torque = 1000.0
        self.basic_x = 1000.0
        self.basic_y = 1000.0
        self.vel_x = 1000.0
        self.vel_y = 1000.0
        self.navigate_to_point = False
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odom_callback,
            10)
            
        self.basic_torque_sub = self.create_subscription(
            Float32,
            'controller/basic_pid/targetTorque',
            self.basic_torque_callback,
            10)           
        self.basic_x_sub = self.create_subscription(
            Float32,
            'controller/basic_pid/targetForceX',
            self.basic_x_callback,
            10)      
        self.basic_y_sub = self.create_subscription(
            Float32,
            'controller/basic_pid/targetForceY',
            self.basic_y_callback,
            10)     
        self.vel_x_sub = self.create_subscription(
            Float32,
            'controller/velocity_pid/targetForceX',
            self.vel_x_callback,
            10)  
        self.vel_y_sub = self.create_subscription(
            Float32,
            'controller/velocity_pid/targetForceY',
            self.vel_y_callback,
            10)  
        self.navigate_to_point_subscriber = self.create_subscription(
            Bool,
            '/controller/navigateToPoint',
            self.navigate_to_point_callback,
            10)     
                        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.1, self.output_data)

    def odom_callback(self, msg):
        self.measured_odom = msg
    
    def basic_torque_callback(self, msg):
        self.basic_torque = msg.data
    
    def basic_x_callback(self, msg):
        self.basic_x = msg.data
        
    def basic_y_callback(self, msg):
        self.basic_y = msg.data
    
    def vel_x_callback(self, msg):
        self.vel_x = msg.data
    
    def vel_y_callback(self, msg):
        self.vel_y = msg.data
    
    def navigate_to_point_callback(self, msg):
        self.navigate_to_point = msg.data
    
    def output_data(self):
    
        pos_x = self.measured_odom.pose.pose.position.x
        pos_y = self.measured_odom.pose.pose.position.y
        
        quat = self.measured_odom.pose.pose.orientation
        
        
        vel_x = self.measured_odom.twist.twist.linear.x        
        vel_y = self.measured_odom.twist.twist.linear.y
        
        angular_vel = self.measured_odom.twist.twist.angular.z*180.0/numpy.pi
        
        heading = [1.0, 0.0, 0.0, 0.0]
        
        q = [self.measured_odom.pose.pose.orientation.x, 
            self.measured_odom.pose.pose.orientation.y, 
            self.measured_odom.pose.pose.orientation.z, 
            self.measured_odom.pose.pose.orientation.w]
                    
        q_inv = q.copy()
        q_inv[0] = -q_inv[0]
        q_inv[1] = -q_inv[1]
        q_inv[2] = -q_inv[2]
                
        heading = tf_transformations.quaternion_multiply(q, heading)
        heading = tf_transformations.quaternion_multiply(heading, q_inv)
        
        heading_angle = numpy.arctan2(heading[1], heading[0])*180.0/numpy.pi
        
        self.get_logger().info('posX: ' + str(pos_x)) 
        self.get_logger().info('posY: ' + str(pos_y))     
        self.get_logger().info('velX: ' + str(vel_x))     
        self.get_logger().info('velY: ' + str(vel_y))     
        self.get_logger().info('angular_vel (d/s): ' + str(angular_vel))     
        self.get_logger().info('heading (d): ' + str(heading_angle))  
        self.get_logger().info('basic_force_x: ' + str(self.basic_x)) 
        self.get_logger().info('basic_force_y: ' + str(self.basic_y)) 
        self.get_logger().info('vel_force_x: ' + str(self.vel_x))         
        self.get_logger().info('vel_force_y: ' + str(self.vel_y))         
        self.get_logger().info('basic_torque: ' + str(self.basic_torque))         
        self.get_logger().info('navigate_to_point: ' + str(self.navigate_to_point))         
                                
def main(args=None):
    rclpy.init(args=args)

    localization_debugger = LocalizationDebuggerNode()

    rclpy.spin(localization_debugger)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization_debugger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
