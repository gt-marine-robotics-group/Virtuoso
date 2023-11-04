import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import ParamVec
from virtuoso_autonomy.utils.task_info import get_state, get_name
from virtuoso_perception.utils.geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class AcousticPerceptionNode(Node):

    def __init__(self):
        super().__init__('acoustic_perception')

        self.state = 'initial'
        self.is_task = False
        self.executing = False

        self.pinger = dict()
        self.pinger_frame = ''

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.task_info_sub = self.create_subscription(ParamVec, '/vrx/task/info', 
            self.task_info_callback, 10)
        
        self.pinger_sub = self.create_subscription(ParamVec, '/wamv/sensors/acoustics/receiver/range_bearing',
            self.pinger_callback, 10)

        self.path_pub = self.create_publisher(Path, '/navigation/set_waypoints', 10)
        
        self.create_timer(1.0, self.execute)

    def task_info_callback(self, msg:ParamVec):
        self.state = get_state(msg)
        self.is_task =  get_name(msg) == 'acoustic_perception'
    
    def pinger_callback(self, msg:ParamVec):
        self.pinger = dict() 
        self.pinger_frame = msg.header.frame_id
        for param in msg.params:
            self.pinger[param.name] = param.value.double_value
    
    def execute(self):

        if not self.is_task:
            return

        if self.executing:
            return

        if len(self.pinger.keys()) == 0:
            return
        
        # self.executing = True
        
        x = math.cos(self.pinger['bearing']) * math.cos(self.pinger['elevation']) * self.pinger['range']
        y = math.sin(self.pinger['bearing']) * math.cos(self.pinger['elevation']) * self.pinger['range']

        trans = None
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom',
                'wamv/wamv/receiver',
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(f'Could not compute transformation from {self.pinger_frame}')
            self.executing = False
            return
        
        p = PoseStamped() 
        p.pose.position.x = x
        p.pose.position.y = y

        self.get_logger().info(f'raw pose: {p}')

        trans_pose = do_transform_pose_stamped(p, trans)

        self.get_logger().info(f'trans pose: {trans_pose}')

        path = Path()
        path.poses.append(trans_pose)
        
        self.path_pub.publish(path)


def main(args=None):
    
    rclpy.init(args=args)

    node = AcousticPerceptionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
