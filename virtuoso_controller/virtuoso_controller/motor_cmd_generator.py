import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import numpy

class MotorCMDGenerator(Node):
    #Generates the motor commands from the information from basic_PID, velocity_PID, and choose_PID

    def __init__(self):
        super().__init__('motor_cmd_generator')
        
        self.state_estimate = Odometry()
        self.destination = Pose()
        self.navigate_to_point = False
        self.received_cmd = False
        self.controller_rate = 0.1
        self.received_basic = False
        self.received_vel = False
        
        self.navigate_to_point_subscriber = self.create_subscription(
            Bool,
            '/controller/navigateToPoint',
            self.navigate_to_point_callback,
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
        self.basic_torque_sub = self.create_subscription(
            Float32,
            'controller/basic_pid/targetTorque',
            self.basic_torque_callback,
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
        self.vel_torque_sub = self.create_subscription(
            Float32,
            'controller/velocity_pid/targetTorque',
            self.vel_torque_callback,
            10)  

        self.left_front_pub_angle = self.create_publisher(Float32, 
            '/wamv/thrusters/left_front_thrust_angle', 10)
        self.right_front_pub_angle = self.create_publisher(Float32, 
            '/wamv/thrusters/right_front_thrust_angle', 10)
        self.left_rear_pub_angle = self.create_publisher(Float32, 
            '/wamv/thrusters/left_rear_thrust_angle', 10)
        self.right_rear_pub_angle = self.create_publisher(Float32, 
            '/wamv/thrusters/right_rear_thrust_angle', 10)

        self.left_front_pub_cmd = self.create_publisher(Float32, 
            '/wamv/thrusters/left_front_thrust_cmd', 10)
        self.right_front_pub_cmd = self.create_publisher(Float32, 
            '/wamv/thrusters/right_front_thrust_cmd', 10)             
        self.left_rear_pub_cmd = self.create_publisher(Float32, 
            '/wamv/thrusters/left_rear_thrust_cmd', 10)
        self.right_rear_pub_cmd = self.create_publisher(Float32, 
            '/wamv/thrusters/right_rear_thrust_cmd', 10)
        
        self.timer = self.create_timer(self.controller_rate, self.timer_callback)

        self.declare_parameter('sim_time', False)

    def navigate_to_point_callback(self, msg:Bool):
        self.navigate_to_point = msg.data

    def basic_x_callback(self, msg:Float32):
        self.basic_force_x = msg.data
        self.received_basic = True
        self.check_received_cmd()

    def basic_y_callback(self, msg:Float32):
        self.basic_force_y = msg.data

    def basic_torque_callback(self, msg:Float32):
        self.basic_torque = msg.data

    def vel_x_callback(self, msg:Float32):
        self.vel_force_x = msg.data
        self.received_vel = True
        self.check_received_cmd()

    def vel_y_callback(self, msg:Float32):
        self.vel_force_y = msg.data              

    def vel_torque_callback(self, msg:Float32):
        self.velTorque = msg.data

    def check_received_cmd(self):
        if(self.received_basic and self.received_vel):
            self.received_cmd = True
        
    def timer_callback(self):
        if not (self.received_cmd and self.received_basic and self.received_vel):
            return

        left_front_angle = Float32()
        right_rear_angle = Float32()
        right_front_angle = Float32()      
        left_rear_angle = Float32()
        
        left_front_cmd = Float32()
        right_rear_cmd = Float32()
        left_rear_cmd = Float32()
        right_front_cmd = Float32()

        left_front_angle.data = -90*numpy.pi/180*0
        right_rear_angle.data = 90*numpy.pi/180*0
        right_front_angle.data = 90*numpy.pi/180*0
        left_rear_angle.data = -90*numpy.pi/180*0

        if (self.navigate_to_point):
                target_force_x = self.basic_force_x
                target_force_y = self.basic_force_y
        else:
                target_force_x = self.vel_force_x
                target_force_y = self.vel_force_y
        target_torque = self.basic_torque
        
                
        left_front_cmd.data = (-target_force_y + target_force_x - target_torque)
        right_front_cmd.data = (target_force_y + target_force_x + target_torque)
        left_rear_cmd.data = (target_force_y*0.6 + target_force_x - target_torque)
        right_rear_cmd.data = (-target_force_y*0.6 + target_force_x + target_torque)
        
        highest_cmd = max(
            abs(left_front_cmd.data), 
            abs(right_front_cmd.data), 
            abs(left_rear_cmd.data), 
            abs(right_rear_cmd.data)
        )
        
        if(highest_cmd > 1.0):
            left_front_cmd.data /= highest_cmd
            right_front_cmd.data /= highest_cmd
            left_rear_cmd.data /= highest_cmd
            right_rear_cmd.data /= highest_cmd
        
        
        if self.get_parameter('sim_time').value:
            if(left_front_cmd.data < 0):
                    left_front_cmd.data *= 2.5
            if(right_front_cmd.data < 0):
                    right_front_cmd.data *= 2.5
            if(left_rear_cmd.data < 0):
                    left_rear_cmd.data *= 2.5
            if(right_rear_cmd.data < 0):
                    right_rear_cmd.data *= 2.5
                                                            
        self.right_front_pub_angle.publish(right_front_angle)
        self.left_rear_pub_angle.publish(left_rear_angle)
        self.left_front_pub_angle.publish(left_front_angle)
        self.right_rear_pub_angle.publish(right_rear_angle)     
    
        self.left_front_pub_cmd.publish(left_front_cmd)
        self.right_rear_pub_cmd.publish(right_rear_cmd)      
        self.right_front_pub_cmd.publish(right_front_cmd)
        self.left_rear_pub_cmd.publish(left_rear_cmd)     

        
def main(args=None):
    rclpy.init(args=args)

    motor_cmd_generator = MotorCMDGenerator()

    rclpy.spin(motor_cmd_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_cmd_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
