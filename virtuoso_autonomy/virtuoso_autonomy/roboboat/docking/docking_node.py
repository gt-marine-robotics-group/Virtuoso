import rclpy
from rclpy.node import Node
from .docking_states import State
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Point, Pose
from virtuoso_msgs.srv import DockCodesCameraPos
import time

TARGET_COLOR = 'red'

class DockingNode(Node):

    def __init__(self):
        super().__init__('autonomy_docking')

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
        self.trans_pub = self.create_publisher(Point, '/navigation/translate', 10)
        self.station_keeping_pub = self.create_publisher(Empty, '/navigation/station_keep', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)

        self.code_pos_client = self.create_client(DockCodesCameraPos, 'find_dock_placard_offsets')
        self.code_pos_req = None
        
        self.state = State.START

        self.robot_pose:Pose = None

        self.create_timer(1.0, self.execute)
    
    def odom_callback(self, msg:Odometry):
        self.robot_pose = msg.pose.pose
    
    def nav_success_callback(self, msg:PoseStamped):
        if self.state == State.APPROACHING_DOCK: self.state = State.ORIENTING
        elif self.state == State.SEARCH_TRANSLATING: self.state = State.SEARCHING_FOR_DOCK_CODE
        elif self.state == State.CENTER_TRANSLATING: self.state = State.CENTERING
        elif self.state == State.DOCKING: self.state = State.COUNTING_CODE
        time.sleep(1.0)
    
    def execute(self):
        self.get_logger().info(str(self.state))
        if self.state == State.START:
            self.enable_station_keeping()
            return
        if self.state == State.STATION_KEEPING_ENABLED:
            # skip the next 2 states until implemented
            self.state = State.SEARCHING_FOR_DOCK_CODE
            return
        if self.state == State.APPROACHING_DOCK:
            return
        if self.state == State.ORIENTING:
            return
        if self.state == State.SEARCHING_FOR_DOCK_CODE:
            self.search_for_dock_code()
            return
        if self.state == State.SEARCH_TRANSLATING:
            return
        if self.state == State.CENTERING:
            self.search_for_dock_code()
            return
        if self.state == State.CENTER_TRANSLATING:
            return
    
    def enable_station_keeping(self):
        self.state = State.STATION_KEEPING_ENABLED
        self.station_keeping_pub.publish(Empty())
    
    def search_for_dock_code(self):
        if self.code_pos_req is not None:
            return

        msg = DockCodesCameraPos.Request()

        self.code_pos_req = self.code_pos_client.call_async(msg)        
        self.code_pos_req.add_done_callback(self.code_pos_callback)
    
    def code_pos_callback(self, future):
        result = future.result()

        self.code_pos_req = None

        self.get_logger().info(f'result: {result}')

        if len(result.red) == 0:
            self.get_logger().info(f'No dock code positions sent')
            return
        
        targetX = -1
        othersX = 0
        othersCount = 0

        if result.red[0] != -1 and result.red[1] != -1:
            mid = (result.red[0] + result.red[1]) // 2
            if TARGET_COLOR == 'red':
                targetX = mid
            else:
                othersX += mid
                othersCount += 1
        
        if result.blue[0] != -1 and result.blue[1] != -1:
            mid = (result.blue[0] + result.blue[1]) // 2
            if TARGET_COLOR == 'blue':
                targetX = mid
            else:
                othersX += mid
                othersCount += 1
        
        if result.green[0] != -1 and result.green[1] != -1:
            mid = (result.green[0] + result.green[1]) // 2
            if TARGET_COLOR == 'green':
                targetX = mid
            else:
                othersX += mid
                othersCount += 1
        
        if othersCount > 0:
            othersX //= othersCount
        
        if self.state == State.CENTERING and targetX == -1:
            self.get_logger().info('Centering but code not found.')
            return
        
        if targetX > -1:
            self.center_translate(targetX, result.image_width)
            return
        
        if othersCount == 0:
            self.get_logger().info('No code found in search.')
            return
        
        self.search_translate(othersX, result.image_width)
    
    def center_translate(self, targetX, image_width):
        msg = Point()

        self.state = State.CENTER_TRANSLATING

        if targetX >= image_width * .45 and targetX <= image_width * .55:
            self.get_logger().info('Within bounds, good to dock')
            self.state = State.DOCKING
            return
        
        if targetX >= image_width * .2 and targetX < image_width * .45:
            self.get_logger().info('going left')
            msg.y = 1.0
        elif targetX < image_width * .2:
            self.get_logger().info('going left')
            msg.y = 2.0
        elif targetX > image_width * .55 and targetX <= image_width * .8:
            self.get_logger().info('going right')
            msg.y = -1.0
        else:
            self.get_logger().info('going right')
            msg.y = -2.0

        self.trans_pub.publish(msg)
    
    def search_translate(self, othersX, image_width):
        msg = Point()

        self.state = State.SEARCH_TRANSLATING

        if othersX < image_width * .5:
            msg.y = 1.0
        else:
            msg.y = -1.0

        self.trans_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = DockingNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()