import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Pose, PoseStamped, Quaternion, Point
from nav_msgs.msg import Odometry, Path
from virtuoso_msgs.srv import Rotate
import tf_transformations

class RotateNode(Node):

    def __init__(self):
        super().__init__('navigation_rotate')

        self.rotate_srv = self.create_service(Rotate, 'rotate', 
            self.rotate_callback)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry',
            self.odom_callback, 10)
        
        self.path_pub = self.create_publisher(Path, '/navigation/plan', 10)

        self.odom:Odometry = None
    
    def odom_callback(self, msg:Odometry):
        self.odom = msg
    
    def rotate_callback(self, req:Rotate.Request, res:Rotate.Response):
        self.get_logger().info('ROTATE CALLBACK')
        res.success = False
        res.failure = False

        if self.odom is None:
            self.get_logger().info('NO ODOM')
            res.failure = True
            return res 
        
        rq = self.odom.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([
            rq.x, rq.y, rq.z, rq.w
        ])

        rq_rotated = tf_transformations.quaternion_from_euler(
            euler[0], euler[1], euler[2] + req.goal.z
        )

        goal_pose = Pose(
            position=self.odom.pose.pose.position,
            orientation=Quaternion(
                x=rq_rotated[0], y=rq_rotated[1], z=rq_rotated[2], 
                w=rq_rotated[3]
            )
        )

        path = Path()
        path.poses.append(PoseStamped(pose=goal_pose))

        self.get_logger().info('SENDING ROTATE PATH')
        self.path_pub.publish(path)

        res.success = True
        return res

def main(args=None):
    rclpy.init(args=args)

    node = RotateNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()