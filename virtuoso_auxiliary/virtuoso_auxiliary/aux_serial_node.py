import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32

class AuxSerialNode(Node):

    def __init__(self):
        super().__init__('auxiliary_aux_serial')

        self.declare_parameters(namespace='', parameters=[
            ('serial_port', '')
        ])

        self.ser = serial.Serial(self.get_parameter('serial_port').value)

        self.water_sub = self.create_subscription(Float32, '/water_shooter/throttle_cmd',
            self.water_callback, 10)
        self.water_cmd = 0.0

        self.a_sub = self.create_subscription(Float32, '/ball_shooter/throttle_a_cmd', 
            self.a_callback, 10)
        self.a_cmd = 0.0

        self.b_sub = self.create_subscription(Float32, '/ball_shooter/throttle_b_cmd', 
            self.b_callback, 10)
        self.b_cmd = 0.0

        self.create_timer(1.0, self.execute)
    
    def water_callback(self, msg:Float32):
        self.water_cmd = msg.data
    
    def a_callback(self, msg:Float32):
        self.a_cmd = msg.data
    
    def b_callback(self, msg:Float32):
        self.b_cmd = msg.data
    
    def execute(self):
        
        s = '5555'
        s += str(int(self.a_cmd * 1000))[:4]
        s += str(int(self.b_cmd * 1000))[:4]
        s += str(int(self.water_cmd * 1000))[:4]
        s += '8888'

        self.get_logger().info(f'Sending over serial: {s}')

        self.ser.write(s)


def main(args=None):
    rclpy.init(args=args)

    node = AuxSerialNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()