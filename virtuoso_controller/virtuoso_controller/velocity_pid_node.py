import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
import tf_transformations
from .velocity_pid import VelocityPID

#Outputs target forces to drive vehicle to target cmd_vel

class VelocityPIDNode(Node):

    def __init__(self):
        super().__init__('controller_velocity_PID')

        self.declare_parameters(namespace='', parameters=[
            ('velocity_k_drag_x', 1.0),
            ('velocity_k_error_x', 1.0),
            ('velocity_ki_x', 1.0),
            ('velocity_k_drag_y', 1.0),
            ('velocity_k_error_y', 1.0),
            ('velocity_ki_y', 1.0)
        ])

        self.pid = VelocityPID(k_drag_x=self.get_parameter('velocity_k_drag_x').value,
            k_error_x=self.get_parameter('velocity_k_error_x').value, 
            ki_x=self.get_parameter('velocity_ki_x').value,
            k_drag_y=self.get_parameter('velocity_k_drag_y').value,
            k_error_y=self.get_parameter('velocity_k_error_y').value, 
            ki_y=self.get_parameter('velocity_ki_y').value)

        self.target_force_x_pub = self.create_publisher(Float32, 
            '/controller/velocity_pid/targetForceX', 10)
        self.target_force_y_pub = self.create_publisher(Float32, 
            '/controller/velocity_pid/targetForceY', 10)

        self.control_mode = 'waypointing'

        self.control_mode_sub =  self.create_subscription(String, 'controller_mode', 
            self.control_mode_callback, 10)
        
        #subscribe to odometry from localization
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odometry_callback,
            10)   
            
        #subscribe to command velocity - in base_link frame
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/controller/cmd_vel',
            self.cmd_vel_callback,
            10)    
        
        self.manual_cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/controller/manual/cmd_vel',
            self.manual_cmd_vel_callback,
            10
        )

        self.create_timer(0.1, self.run_pid)
    
    def control_mode_callback(self, msg:String):
        self.control_mode = msg.data
        
    def odometry_callback(self, msg:Odometry):
        self.pid.state_estimate = msg
    
    def cmd_vel_callback(self, msg:Twist):
        if self.control_mode != 'waypointing': return

        self.pid.target_twist = msg
        self.pid.received_cmd_vel = True
    
    def manual_cmd_vel_callback(self, msg:Twist):
        if self.control_mode != 'manual': return

        self.pid.target_twist = msg
        self.pid.received_cmd_vel = True

    def run_pid(self): 
        x, y = self.pid.run()
        
        if x is None:
            return
        
        self.target_force_x_pub.publish(x)
        self.target_force_y_pub.publish(y)
        

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
