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

        self.prev_str = None
        self.prev_count = 0

        self.get_logger().info(str(self.get_parameter('serial_port').value))

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

        self.create_timer(0.05, self.execute)
    
    def water_callback(self, msg:Float32):
        self.water_cmd = msg.data
    
    def a_callback(self, msg:Float32):
        self.a_cmd = msg.data
    
    def b_callback(self, msg:Float32):
        self.b_cmd = msg.data
    
    def execute(self):

        self.prev_count += 1
        
        s = 'AAAA'
        s += self.num_str(abs(self.a_cmd))
        s += self.num_str(abs(self.b_cmd))
        s += self.num_str(abs(self.water_cmd))
        s += 'BBBB'

        s = s.encode('ascii')

        if self.prev_str == s and self.prev_count < 10:
            return
        
        self.prev_str = s
        self.prev_count = 0

        self.get_logger().info(f'Sending over serial: {s}')

        self.ser.write(s)
    
    def num_str(self, val:float):
        s = str(str(int(val * 1000)))
        if len(s) < 4:
            s = (str(0) * (4 - len(s))) + s
        return s[:4]


def main(args=None):
    rclpy.init(args=args)

    node = AuxSerialNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
