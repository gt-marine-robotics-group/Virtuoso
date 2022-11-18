import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from tf2_ros.buffer import Buffer
from rclpy.time import Time, Duration
from tf2_ros.transform_listener import TransformListener
import time
from virtuoso_perception.utils.geometry_msgs import do_transform_pose_stamped

class TranslateNode(Node):

    def __init__(self):
        super().__init__('navigation_translate') # using namespace interferes with tf listener

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self._action_server = ActionServer(self, Translate, '/navigation/translate', self.execute_callback)
        self.translate_sub = self.create_subscription(Point, '/navigation/translate', 
            self.execute_callback, 10)
        self.nav_success_sub = self.create_subscription(PoseStamped, '/virtuoso_navigation/success',
            self.nav_success_callback, 10)

        # self.set_path_pub = self.create_publisher(Path, '/navigation/set_trans_path', 10)
        self.set_path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)
        # self.set_path_pub = self.create_publisher(Path, '/transformed_global_plan', 10)
        self.translate_success_pub = self.create_publisher(Point, '/navigation/translate_success',
            10)
        
        self.goal = None
        self.trans_goal = None
    
    def nav_success_callback(self, msg:PoseStamped):
        if self.trans_goal is None:
            return
        if msg.pose.position.x != self.trans_goal.pose.position.x:
            return
        if msg.pose.position.y != self.trans_goal.pose.position.y:
            return
        self.get_logger().info('Completed Translation')
        self.translate_success_pub.publish(self.goal)
    
    def send_path(self):
        
        trans = None

        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform('map', 'wamv/base_link', now)
        except Exception as e:
            self.get_logger().info('FAILED TRANSFORM')
            self.get_logger().info(str(e))
            return
        
        path = Path()
        ps = PoseStamped()
        ps.pose.position.x = self.goal.x
        ps.pose.position.y = self.goal.y

        trans_point = do_transform_pose_stamped(ps, trans)
        path.poses.append(trans_point)

        self.trans_goal = trans_point

        self.set_path_pub.publish(path)

    def execute_callback(self, msg:Point):
        self.goal = msg
        self.send_path()


def main(args=None):
    rclpy.init(args=args)

    translate_node = TranslateNode()

    rclpy.spin(translate_node)

if __name__ == '__main__':
    main()