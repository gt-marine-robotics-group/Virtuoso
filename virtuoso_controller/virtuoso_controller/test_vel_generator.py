import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

#import pyproj
#import math
import numpy

class testWaypointGenerator(Node):

    def __init__(self):
        super().__init__('test_waypoint_generator')
        

        self.targetTwistStamped = TwistStamped()
        self.targetTwist = Twist()
        self.targetTwist.linear.x = 5.0
        self.targetTwist.linear.y = 5.0        
        self.targetTwist.angular.z = 0.0
        self.targetTwistStamped.twist = self.targetTwist
	
        self.waypointPub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        self.waypointPub.publish(self.targetTwistStamped)

       
     
    

        
def main(args=None):
    rclpy.init(args=args)

    test_waypoint_generator = testWaypointGenerator()

    rclpy.spin(test_waypoint_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_waypoint_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
