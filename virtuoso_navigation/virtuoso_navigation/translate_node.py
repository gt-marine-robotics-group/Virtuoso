import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from virtuoso_msgs.action import Translate
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer
from rclpy.time import Time, Duration
from tf2_ros.transform_listener import TransformListener
import time
from virtuoso_perception.utils.geometry_msgs import do_transform_pose_stamped

class TranslateActionServer(Node):

    def __init__(self):
        super().__init__('navigation_translate') # using namespace interferes with tf listener

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_server = ActionServer(self, Translate, '/navigation/translate', self.execute_callback)

        self.set_path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/virtuoso_navigation/success',
            self.nav_success_callback, 10)
        
        self.goal_handle = None
        self.nav_success = False
        self.aborted_result = None
    
    def nav_success_callback(self, msg):
        self.nav_success = True
    
    def send_path(self):
        
        trans = None

        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform('map', 'wamv/base_link', now)
        except Exception as e:
            self.get_logger().info('FAILED TRANSFORM')
            self.get_logger().info(str(e))
            self.goal_handle.abort()
            self.aborted_result = Translate.Result()
            self.aborted_result.success = 0
            return
        
        path = Path()
        ps = PoseStamped()
        ps.pose.position.x = self.goal_handle.request.x
        ps.pose.position.y = self.goal_handle.request.y

        trans_point = do_transform_pose_stamped(ps, trans)
        path.poses.append(trans_point)

        self.set_path_pub.publish(path)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing action')
        self.get_logger().info(f'{goal_handle.request.x}, {goal_handle.request.y}')

        self.nav_success = False
        self.aborted_result = None
        self.goal_handle = goal_handle

        # self.send_path(goal_handle.request.x, goal_handle.request.y)
        self.send_path()

        while not self.nav_success:
            if self.aborted_result is not None: 
                return self.aborted_result
            time.sleep(0.1)

        self.get_logger().info('SUCCESS')
        goal_handle.succeed()
        result = Translate.Result()
        result.success = 1
        return result


def main(args=None):
    rclpy.init(args=args)

    translate_action_server = TranslateActionServer()

    rclpy.spin(translate_action_server)

if __name__ == '__main__':
    main()