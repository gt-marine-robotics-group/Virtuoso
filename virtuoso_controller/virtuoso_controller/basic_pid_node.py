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
