import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped

#import pyproj
#import math
import numpy

class testWaypointGenerator(Node):

    def __init__(self):
        super().__init__('test_waypoint_generator')
        

        self.targetWaypoint = GeoPoseStamped()
        self.targetWaypoint.pose.position.latitude = -33.722767309712644 + 0.0006
        self.targetWaypoint.pose.position.longitude = 150.67399020302886 + 0.0006
	
        self.waypointPub = self.create_publisher(GeoPoseStamped, '/vrx/station_keeping/goal', 10)
        timer_period = 10.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        self.waypointPub.publish(self.targetWaypoint)

       
     
    

        
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
