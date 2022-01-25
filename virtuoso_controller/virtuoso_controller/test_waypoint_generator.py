import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

#import pyproj
#import math
import numpy

class testWaypointGenerator(Node):

    def __init__(self):
        super().__init__('test_waypoint_generator')
        

        self.targetWaypoint = Odometry()
        self.targetPose = Pose()
        self.targetPose.position.x = 15.0
        self.targetPose.position.y = 10.0        
        self.targetPose.orientation.z = numpy.sin(180*numpy.pi/180/2)
        self.targetPose.orientation.w = numpy.cos(180*numpy.pi/180/2)
        self.targetWaypoint.pose.pose = self.targetPose
	
        self.waypointPub = self.create_publisher(Odometry, '/waypoint', 10)
        timer_period = 0.05  # seconds
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
