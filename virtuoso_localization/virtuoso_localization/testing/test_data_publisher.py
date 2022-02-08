import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header
#from builtin_interfaces.msg import Time

class testPublisher(Node):

    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(Imu, '/wamv/sensors/imu/imu/data', 10)
        self.publisher2_ = self.create_publisher(NavSatFix, '/wamv/sensors/gps/gps/fix', 10)
        self.publisher3_ = self.create_publisher(Vector3Stamped, '/wamv/sensors/gps/gps/fix_velocity', 10)
        #self.publisher4_ = self.create_publisher(Time, '/clock', 10)
        timer_period = 0.1  # seconds
        self.time = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = Imu()
        header = Header()
        header.frame_id = "wamv/imu_wamv_link"
        header.stamp = self.get_clock().now().to_msg()
        msg.header = header
        msg2 = NavSatFix()
        header2 = Header()
        header2.stamp = self.get_clock().now().to_msg()
        header2.frame_id = "wamv/gps_wamv_link"
        msg2.header = header2
        msg3 = Vector3Stamped()
        self.publisher_.publish(msg)
        self.publisher2_.publish(msg2)
        self.publisher3_.publish(msg3)
        #self.publisher4_.publish(self.get_clock().now().to_msg())


def main(args=None):
    rclpy.init(args=args)

    test_publisher = testPublisher()

    rclpy.spin(test_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
