import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Pose, PoseStamped, Quaternion, Point
from nav_msgs.msg import Odometry, Path
from virtuoso_msgs.srv import Rotate
import tf_transformations
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

class RotateNode(Node):

    def __init__(self):
        super().__init__('navigation_rotate')

        self.declare_parameter('goal_tolerance', 0.0)

        self.cb_group1 = MutuallyExclusiveCallbackGroup()
        self.cb_group2 = MutuallyExclusiveCallbackGroup()

        self.rotate_srv = self.create_service(Rotate, 'rotate', 
            self.rotate_callback, callback_group=self.cb_group1)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry',
            self.odom_callback, 10, callback_group=self.cb_group2)
        
        self.path_pub = self.create_publisher(Path, '/navigation/plan', 10)

        self.odom:Odometry = None
        self.goal_yaw:float = None
    
    def odom_callback(self, msg:Odometry):
        self.odom = msg
    
    def check_goal_reached(self):
        rq = self.odom.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([
            rq.x, rq.y, rq.z, rq.w
        ])

        if abs(self.goal_yaw - euler[2]) <= self.get_parameter('goal_tolerance').value:
            return True
        
        return False

    def rotate_callback(self, req:Rotate.Request, res:Rotate.Response):
        res.success = False
        res.failure = False

        if self.odom is None:
            res.failure = True
            return res 
        
        rq = self.odom.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([
            rq.x, rq.y, rq.z, rq.w
        ])

        self.goal_yaw = euler[2] + req.goal.z

        rq_rotated = tf_transformations.quaternion_from_euler(
            euler[0], euler[1], self.goal_yaw
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

        self.path_pub.publish(path)

        while not self.check_goal_reached():
            self.get_logger().info('Rotating...')
            time.sleep(0.5)

        res.success = True
        return res

def main(args=None):
    rclpy.init(args=args)

    node = RotateNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    # rclpy.spin(node)

    # node.destroy_node()
    rclpy.shutdown()