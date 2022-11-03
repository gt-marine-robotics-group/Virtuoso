import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2
from virtuoso_processing.utils.pointcloud import read_points
import time

class DockingNode(Node):

    def __init__(self):
        super().__init__('autonomy_docking', namespace='autonomy')

        self.target_dock_color = 'red'

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
        
        self.trans_pub = self.create_publisher(Point, '/navigation/translate', 10)
        self.trans_success_sub = self.create_subscription(Point, '/navigation/translate_success',
            self.trans_success_callback, 10)
        
        self.odom:Odometry = None
        self.find_dock_codes_ready = False
        self.find_dock_entrances_ready = False
        self.station_keeping_enabled = False
        self.find_docks_req_sent = False

        # self.target_offset = 0
        self.color_docks = dict() # map of color => index
        self.entrances = list() # list of 4 points (x, y)
        self.translating = False
        self.translate_complete = False
        self.entering = False
        self.at_entrance = False

        self.create_timer(1.0, self.send_find_docks_req)
        self.create_timer(1.0, self.go_to_entrance)
    
    def odom_callback(self, msg):
        self.odom = msg
        if not self.station_keeping_enabled:
            self.enable_station_keeping()
    
    # def find_docks_ready_callback(self, msg):
    #     self.find_docks_ready = True
    def find_dock_codes_ready_callback(self, msg):
        self.find_dock_codes_ready = True
    
    def find_dock_entrances_ready_callback(self, msg):
        self.find_dock_entrances_ready = True
    
    def enable_station_keeping(self):
        path = Path()
        pose_stamped = PoseStamped()
        pose_stamped.pose = self.odom.pose.pose
        path.poses.append(pose_stamped)
        self.path_pub.publish(path)
        self.station_keeping_enabled = True
        self.get_logger().info('Station Keeping Enabled')
    
    def send_find_docks_req(self):
        if not self.station_keeping_enabled:
            return
        
        if not self.find_dock_entrances_ready or not self.find_dock_codes_ready:
            return
        
        if self.find_docks_req_sent:
            return
        
        self.get_logger().info('Sending Find Docks Request')
        self.find_docks_req_sent = True
        self.find_docks_req_pub.publish(Int8(data=1))
    
    def offsets_callback(self, msg:Int32MultiArray):
        if 1 in msg.data[3:6]:
            return
        if self.translating:
            return
        
        self.get_logger().info(str(msg.data[0:3]))

        offset_to_color = dict()
        offset_to_color[msg.data[0]] = 'red'
        offset_to_color[msg.data[1]] = 'green'
        offset_to_color[msg.data[2]] = 'blue'

        offsets = list(msg.data[0:3])
        offsets.sort()

        for i, offset in enumerate(offsets):
            self.color_docks[offset_to_color[offset]] = i

        self.get_logger().info(str(self.color_docks))

        if not self.translating:
            self.translate()
    
    def entrances_callback(self, msg:PointCloud2):
        self.entrances = list()
        for point in read_points(msg):
            self.entrances.append((point[0], point[1]))
    
    def find_mid(self):
        index = self.color_docks[self.target_dock_color] 
        p1 = self.entrances[index]
        p2 = self.entrances[index + 1]

        return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    
    def translate(self):
        if not self.color_docks:
            return
        if len(self.entrances) == 0:
            return
        
        self.get_logger().info(str(self.entrances))
        self.translating = True

        mid = self.find_mid()

        self.trans_pub.publish(Point(x=mid[0]-5, y=mid[1]))

    def trans_success_callback(self, msg):
        self.translate_complete = True
        if self.entering and not self.at_entrance:
            self.enter_dock()
            return

        self.entrances = list()
        if not self.entering:
            time.sleep(10.0) 
            # self.go_to_entrance()
        # self.translating = False
    
    def go_to_entrance(self):
        if self.entering:
            return
        if not self.translate_complete:
            return
        if len(self.entrances) < 4:
            return

        self.entering = True

        mid = self.find_mid()

        self.trans_pub.publish(Point(x=mid[0], y=mid[1]))
    
    def enter_dock(self):
        self.trans_pub.publish(Point(x=2.0)) 

def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()