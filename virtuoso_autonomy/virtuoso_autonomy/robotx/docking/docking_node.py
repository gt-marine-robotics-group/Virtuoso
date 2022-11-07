import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2
from virtuoso_processing.utils.pointcloud import read_points
import time
from .docking_states import State
import numpy as np

class DockingNode(Node):

    def __init__(self):
        super().__init__('autonomy_docking')

        self.declare_parameters(namespace='', parameters=[
            ('target_dock_color', 'red')
        ])

        self.target_dock_color = self.get_parameter('target_dock_color').value

        self.find_docks_req_pub = self.create_publisher(Int8, '/perception/start_find_docks', 10)
        self.path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)

        self.find_docks_ready_sub = self.create_subscription(Int8, '/perception/find_dock_codes/ready',
            self.find_dock_codes_ready_callback, 10)
        self.find_dock_entrances_ready_sub = self.create_subscription(Int8, 
            '/perception/find_dock_entrances/ready', self.find_dock_entrances_ready_callback, 10)

        self.dock_offsets_sub = self.create_subscription(Int32MultiArray, 
            '/perception/dock_code_offsets', self.offsets_callback, 10)
        self.dock_entrances_sub = self.create_subscription(PointCloud2, 
            '/perception/dock_entrances', self.entrances_callback, 10)
        self.dock_ahead_entrace_sub = self.create_subscription(PointCloud2,
            '/perception/dock_ahead_entrance', self.ahead_entrance_callback, 10)
        
        self.trans_pub = self.create_publisher(Point, '/navigation/translate', 10)
        self.trans_success_sub = self.create_subscription(Point, '/navigation/translate_success',
            self.trans_success_callback, 10)
        
        self.odom:Odometry = None
        self.find_dock_codes_ready = False
        self.find_dock_entrances_ready = False
        self.station_keeping_enabled = False
        self.find_docks_req_sent = False

        self.color_docks = dict() # map of color => index
        self.entrances = list() # list of 4 points (x, y)
        self.ahead_entrance = list() # 2 points
        self.state = State.START

        self.create_timer(1.0, self.send_find_docks_req)
        self.create_timer(1.0, self.execute)
    
    def odom_callback(self, msg):
        self.odom = msg
    
    def find_dock_codes_ready_callback(self, msg):
        self.find_dock_codes_ready = True
    
    def find_dock_entrances_ready_callback(self, msg):
        self.find_dock_entrances_ready = True
    
    def execute(self):
        self.get_logger().info(f'State: {self.state}')
        if self.state == State.START:
            self.enable_station_keeping()
            return
        if self.state == State.STATION_KEEPING_ENABLED:
            self.approach()
            return
        if self.state == State.APPROACHING:
            self.ahead_entrance = list()
            return
        if self.state == State.APPROACHING_COMPLETE:
            self.translate_to_dock()
            return
        if self.state == State.TRANSLATING:
            self.ahead_entrance = list()
            return
        if self.state == State.COMPLETED_TRANSLATION:
            self.go_to_entrance()
            return
        if self.state == State.ENTERING_DOCK:
            return
        if self.state == State.AT_ENTERANCE:
            self.enter_dock()
            return
        if self.state == State.ENTERING_FURTHER:
            return
        self.get_logger().info('COMPLETE') 

    
    def enable_station_keeping(self):
        if self.odom is None:
            return
        self.state = State(self.state.value + 1)
        path = Path()
        pose_stamped = PoseStamped()
        pose_stamped.pose = self.odom.pose.pose
        path.poses.append(pose_stamped)
        self.path_pub.publish(path)
        self.get_logger().info('Station Keeping Enabled')
    
    def approach(self):
        if len(self.ahead_entrance) < 2:
            return
        
        self.state = State(self.state.value + 1)

        self.entering = True

        mid = ((self.ahead_entrance[0][0] + self.ahead_entrance[1][0]) / 2, 
            (self.ahead_entrance[0][1] + self.ahead_entrance[1][1]) / 2)
        
        self.trans_pub.publish(Point(x=mid[0]-5, y=mid[1]))
    
    def send_find_docks_req(self):
        
        if not self.find_dock_entrances_ready or not self.find_dock_codes_ready:
            return
        
        if self.find_docks_req_sent:
            return
        
        self.get_logger().info('Sending Find Docks Request')
        self.find_docks_req_sent = True
        self.find_docks_req_pub.publish(Int8(data=1))
    
    def offsets_callback(self, msg:Int32MultiArray):
        if self.state != State.APPROACHING_COMPLETE:
            return
        if 1 in msg.data[3:6]:
            return

        offset_to_color = dict()
        offset_to_color[msg.data[0]] = 'red'
        offset_to_color[msg.data[1]] = 'green'
        offset_to_color[msg.data[2]] = 'blue'

        offsets = list(msg.data[0:3])
        offsets.sort()

        for i, offset in enumerate(offsets):
            self.color_docks[offset_to_color[offset]] = i

    
    def entrances_callback(self, msg:PointCloud2):
        self.entrances = list()
        for point in read_points(msg):
            self.entrances.append((point[0], point[1]))
    
    def ahead_entrance_callback(self, msg:PointCloud2):
        self.ahead_entrance = list()
        for point in read_points(msg):
            self.ahead_entrance.append((point[0], point[1]))
    
    def find_mid(self, p1, p2):
        return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    
    def translate_to_dock(self):
        if not self.color_docks:
            return
        if len(self.ahead_entrance) < 2:
            return
        
        self.state = State(self.state.value + 1)

        p1 = None
        p2 = None
        change = (
            self.ahead_entrance[1][0] - self.ahead_entrance[0][0],
            self.ahead_entrance[1][1] - self.ahead_entrance[0][1]
        )
        index = self.color_docks[self.target_dock_color] 
        if index == 0:
            p1 = self.ahead_entrance[0]
            p2 = np.subtract(p1, change)
        elif index == 1:
            p1 = self.ahead_entrance[0]
            p2 = self.ahead_entrance[1]
        else:
            p1 = self.ahead_entrance[1]
            p2 = np.add(p1, change)
        
        mid = self.find_mid(p1, p2)


        self.trans_pub.publish(Point(x=mid[0]-5, y=mid[1]))

    def trans_success_callback(self, msg):
        self.state = State(self.state.value + 1)
    
    def go_to_entrance(self):
        if len(self.ahead_entrance) < 2:
            return

        self.state = State(self.state.value + 1)

        mid = ((self.ahead_entrance[0][0] + self.ahead_entrance[1][0]) / 2, 
            (self.ahead_entrance[0][1] + self.ahead_entrance[1][1]) / 2)

        self.trans_pub.publish(Point(x=mid[0], y=mid[1]))
    
    def enter_dock(self):
        self.state = State(self.state.value + 1)
        self.trans_pub.publish(Point(x=2.0)) 

def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()