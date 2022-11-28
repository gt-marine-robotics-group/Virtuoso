import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf_transformations
from .velocity_pid import VelocityPID

class VelocityPIDNode(Node):

    def __init__(self):
        super().__init__('controller_velocity_PID')

        self.declare_parameters(namespace='', parameters=[
            ('velocity_kp', 1.0),
            ('velocity_kd', 1.0),
            ('velocity_ki', 1.0)
        ])

        self.pid = VelocityPID(kp=self.get_parameter('velocity_kp').value,
            kd=self.get_parameter('velocity_kd').value, 
            ki=self.get_parameter('velocity_ki').value)

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
            '/controller/cmd_vel',
            self.cmd_vel_callback,
            10)    

        self.create_timer(0.1, self.run_pid)
        
    def odometry_callback(self, msg:Odometry):
        self.pid.state_estimate = msg
    
    def cmd_vel_callback(self, msg:Twist):
        self.pid.target_twist = msg
        self.pid.received_cmd_vel = True

    def run_pid(self): 
        x, y, torque = self.pid.run()
        
        if x is None:
            return
        
        self.target_force_x_pub.publish(x)
        self.target_force_y_pub.publish(y)
        self.target_torque_pub.publish(torque)
        

def main(args=None):
    rclpy.init(args=args)

    velocity_PID = VelocityPIDNode()

    rclpy.spin(velocity_PID)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_PID.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
