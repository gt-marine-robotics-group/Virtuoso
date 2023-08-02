import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32, Float64
from .motor_cmd_generator import MotorCmdGenerator

class MotorCmdGeneratorNode(Node):
    #Generates the motor commands from the information from basic_PID, velocity_PID, and choose_PID

    def __init__(self):
        super().__init__('controller_motor_cmd_generator')

        self.declare_parameters(namespace='', parameters=[
            ('sim_time', False),
            ('motor_config', "X"),
            ('motors_general', ['']),
            ('motor_angle_topics', ['']),
            ('motor_cmd_topics', [''])
        ])

        self.generator = MotorCmdGenerator(sim_time=self.get_parameter('sim_time').value, motor_config=self.get_parameter('motor_config').value)
        
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
        
        self.pubs = dict() 
        for i, motor in enumerate(self.get_parameter('motors_general').value):
            self.pubs[f'{motor}_angle'] = self.create_publisher(Float64,
                self.get_parameter('motor_angle_topics').value[i], 10)
            self.pubs[f'{motor}_cmd'] = self.create_publisher(Float64,
                self.get_parameter('motor_cmd_topics').value[i], 10)
                
        self.create_timer(0.1, self.timer_callback)

    def navigate_to_point_callback(self, msg:Bool):
        self.generator.navigate_to_point = msg.data

    def basic_x_callback(self, msg:Float32):
        self.generator.basic_force_x = msg.data

    def basic_y_callback(self, msg:Float32):
        self.generator.basic_force_y = msg.data

    def basic_torque_callback(self, msg:Float32):
        self.generator.basic_torque = msg.data

    def vel_x_callback(self, msg:Float32):
        self.generator.vel_force_x = msg.data

    def vel_y_callback(self, msg:Float32):
        self.generator.vel_force_y = msg.data
        
    def timer_callback(self):
        commands = self.generator.run()

        if commands is None:
            return
        
        for key, value in commands.items():
            self.pubs[key].publish(value)
        
def main(args=None):
    rclpy.init(args=args)

    motor_cmd_generator = MotorCmdGeneratorNode()

    rclpy.spin(motor_cmd_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_cmd_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
