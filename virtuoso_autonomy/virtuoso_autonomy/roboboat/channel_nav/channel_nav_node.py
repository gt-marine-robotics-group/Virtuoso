import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from virtuoso_msgs.msg import YOLOResultArray
from sensor_msgs.msg import CameraInfo

class ChannelNavNode(Node):

    def __init__(self):
        super().__init__('channel_nav')

        self.control_mode_pub = self.create_publisher(String, 'controller_mode', 10)

        self.vel_pub = self.create_publisher(Twist, '/controller/manual/cmd_vel', 10)
        self.torque_pub = self.create_publisher(Float32, '/controller/manual/cmd_torque', 10)

        self.yolo_sub = self.create_subscription(YOLOResultArray, 'yolo_results', 
            self.yolo_callback, 10)
        
        self.cam_info_sub = self.create_subscription(CameraInfo, 'camera_info', 
            self.cam_info_callback, 10)

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.control_mode_is_set = False

        self.yolo_results = None
        self.cam_info = None
    
    def cam_info_callback(self, msg: CameraInfo):
        self.cam_info = msg
    
    def yolo_callback(self, msg: YOLOResultArray):
        self.yolo_results = msg 
    
    def timer_callback(self):
        # if not self.control_mode_is_set:
        self.control_mode_pub.publish(String(data='manual'))
            # self.control_mode_is_set = True
        
        if self.yolo_results is None:
            self.get_logger().info('No YOLO results')
            return
        
        if self.cam_info is None:
            self.get_logger().info('No Camera Info')
            return
        
        # self.get_logger().info(f'results: {self.yolo_results}')

        largest_green = {
            'size': 0,
            'x': 0,
            'y': 0
        }
        largest_red = {
            'size': 0,
            'x': 0,
            'y': 0
        }

        for buoy in self.yolo_results.results:
            if 'green' not in buoy.label and 'red' not in buoy.label: continue

            x = (buoy.x1 + buoy.x2) / 2
            y = (buoy.y1 + buoy.y2) / 2
            size = abs(buoy.x1 - buoy.x2) * abs(buoy.y1 - buoy.y2)

            if 'green' in buoy.label:
                if size > largest_green['size']:
                    largest_green = {'x': x, 'y': y, 'size': size}
            
            if 'red' in buoy.label:
                if size > largest_red['size']:
                    largest_red = {'x': x, 'y': y, 'size': size}
        
        if largest_red is None and largest_green is None:
            self.get_logger().info('Could not find red or green buoy')
            return

        cam_size = self.cam_info.width * self.cam_info.height
        
        twist = Twist()
        torque = Float32()

        if largest_green['size'] == 0:
            self.get_logger().info('largest green none')
            torque.data = -1.0
            twist.linear.y =  -1.0
            twist.linear.x = largest_red['size'] / (cam_size / 4)
        elif largest_red['size'] == 0:
            self.get_logger().info('largest red none')
            torque.data = 1.0
            twist.linear.y = 1.0
            twist.linear.x = largest_green['size'] / (cam_size / 4)
        else:
            horz_red = (self.cam_info.width / 2) - largest_red['x']
            horz_green = largest_green['x'] - (self.cam_info.width / 2)


            torque.data = (horz_red - horz_green) / (self.cam_info.width / 2)
            if horz_red > horz_green:
                twist.linear.y = 1 - (horz_green / horz_red)
            else:
                twist.linear.y = (1 - (horz_red / horz_green)) * -1
            twist.linear.x = (largest_green['size'] + largest_red['size']) / (self.cam_info.width / 2)
        
        torque.data *= -1

        self.torque_pub.publish(torque)
        self.vel_pub.publish(twist)

        self.get_logger().info(f'torque: {torque}')
        self.get_logger().info(f'twist: {twist}')


def main(args=None):
    rclpy.init(args=args)

    node = ChannelNavNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()