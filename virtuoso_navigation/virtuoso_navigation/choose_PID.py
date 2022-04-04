import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

class choosePID(Node):

    def __init__(self):
        super().__init__('choose_PID')
        
        self.stateEstimate = Odometry()
        self.destination = Pose()
        self.navigateToPoint = Bool()
        self.navigateToPoint.data = False
        self.receivedPath = False
        self.nextWaypoint = Pose()
        
        self.path_subscriber = self.create_subscription(
            Path,
            '/transformed_global_plan',
            self.path_callback,
            10)  
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odometry_callback,
            10)   

        self.navigateToPointPub = self.create_publisher(Bool, '/navigation/navigateToPoint', 10)
        self.waypointPub = self.create_publisher(Odometry, '/waypoint', 10)        
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def path_callback(self, msg):
        self.destination = msg.poses[-1].pose
        self.nextWaypoint = msg.poses[0].pose
        self.receivedPath = True
        
    def timer_callback(self):
        if(self.receivedPath):
             destX = self.destination.position.x
             destY = self.destination.position.y
        
             selfX = self.stateEstimate.pose.pose.position.x
             selfY = self.stateEstimate.pose.pose.position.y
        
             distance = ((destX - selfX)**2 + (destY - selfY)**2)**(1/2)
             #self.get_logger().info('distance: ' + str(distance)) 
             if(distance < 2.0):
                  self.navigateToPoint.data = True
             else:
                  self.navigateToPoint.data = False
             self.navigateToPointPub.publish(self.navigateToPoint)
        
             targetWaypoint = Odometry()
             if(distance < 2.0):
                  targetWaypoint.pose.pose = self.destination
             else:
     	          targetWaypoint.pose.pose.position = self.destination.position
     	          targetWaypoint.pose.pose.orientation = self.nextWaypoint.orientation
             #targetWaypoint.pose.pose.orientation = 
             self.waypointPub.publish(targetWaypoint)
  
    def odometry_callback(self, msg):
        self.stateEstimate = msg       

        
def main(args=None):
    rclpy.init(args=args)

    choose_PID = choosePID()

    rclpy.spin(choose_PID)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    choose_PID.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
