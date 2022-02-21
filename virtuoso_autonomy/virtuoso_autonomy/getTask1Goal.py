import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from geodesy import utm
import tf2_ros 
#import tf2_geometry_msgs

class getTask1Goal(Node):

    def __init__(self):
        super().__init__('get_task_1_goal')
        self.receivedGoal = GeoPoseStamped()
        self.goalToSend = PoseStamped()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goalMapPublisher = self.create_publisher(PoseStamped, '/virtuoso_navigation/set_goal', 10)

       
        self.goal_subscriber = self.create_subscription(
            GeoPoseStamped,
            '/vrx/station_keeping/goal',
            self.goal_callback,
            10)          
        self.goal_subscriber

        self.prev_goal = None

        
    def goal_callback(self, msg):
        #msg2 = Imu()
        self.receivedGoal = msg
        geopoint = msg.pose.position
        lat = geopoint.latitude
        lon = geopoint.longitude
        
        goalpoint = utm.fromLatLong(lat, lon)
        
        self.goalToSend.header = Header() 
        self.goalToSend.header.frame_id = "map"
        #self.goalToSend.pose.position.x = goalpoint.easting
        #self.goalToSend.pose.position.y = goalpoint.northing    

        self.goalToSend.pose = Pose()
        
        self.goalToSend.pose.position.x = -5.0
        self.goalToSend.pose.position.y = 0.0
        
        if self.prev_goal is not None and self.prev_goal.pose.position.x == self.goalToSend.pose.position.x and self.prev_goal.pose.position.y == self.goalToSend.pose.position.y:
            return
        """
        self.goalToSend.pose.orientation = msg.pose.orientation   
        try:
                transformation = self.tf_buffer.lookup_transform('map', 'utm', self.get_clock().now())
                goalToSend = tf2_geometry_msgs.do_transform_pose(goalToSend, transformation)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
    	        pass
    	        """
        self.prev_goal = self.goalToSend

        self.goalMapPublisher.publish(self.goalToSend)
 

        
def main(args=None):
    rclpy.init(args=args)

    get_task_1_goal = getTask1Goal()

    rclpy.spin(get_task_1_goal)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_task_1_goal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
