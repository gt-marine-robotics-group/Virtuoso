import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from virtuoso_perception.utils.pointcloud import read_points
from std_msgs.msg import Empty
from .docking_states import State
from .docking import Docking

class DockingNode(Node):

    def __init__(self):
        super().__init__('autonomy_docking')

        self.declare_parameters(namespace='', parameters=[
            ('target_dock_color', 'red'),
            ('dock_approach_dist', 0.0)
        ])

        self.find_docks_req_pub = self.create_publisher(Int8, '/perception/start_find_docks', 10)
        self.path_pub = self.create_publisher(Path, '/navigation/set_waypoints', 10)
        self.station_keeping_pub = self.create_publisher(Empty, '/navigation/station_keep', 10)

        self.find_docks_ready_sub = self.create_subscription(Int8, '/perception/find_dock_codes/ready',
            self.find_dock_codes_ready_callback, 10)

        self.dock_offsets_sub = self.create_subscription(Int32MultiArray, 
            '/perception/dock_code_offsets', self.offsets_callback, 10)
        self.dock_entrances_sub = self.create_subscription(PointCloud2, 
            '/perception/dock_entrances', self.entrances_callback, 10)
        self.dock_ahead_entrace_sub = self.create_subscription(PointCloud2,
            '/perception/dock_ahead_entrance', self.ahead_entrance_callback, 10)
        
        self.trans_pub = self.create_publisher(Point, '/navigation/translate', 10)
        self.trans_success_sub = self.create_subscription(Point, '/navigation/translate_success',
            self.trans_success_callback, 10)
        
        self.find_dock_codes_ready = False
        self.find_dock_entrances_ready = False
        self.find_docks_req_sent = False

        self.docking = Docking(target_dock_color=self.get_parameter('target_dock_color').value,
            dock_approach_dist=self.get_parameter('dock_approach_dist').value)

        self.state = State.START

        self.create_timer(1.0, self.execute)
    
    def find_dock_codes_ready_callback(self, msg):
        self.find_dock_codes_ready = True
    
    def execute(self):
        self.get_logger().info(f'State: {self.state}')
        if self.state == State.START:
            self.enable_station_keeping()
            return
        if self.state == State.STATION_KEEPING_ENABLED:
            self.send_find_docks_req()
            return
        if self.state == State.FINDING_DOCKS:
            self.approach()
            return
        if self.state == State.APPROACHING:
            self.docking.ahead_entrance = list()
            return
        if self.state == State.APPROACHING_COMPLETE:
            self.translate_to_dock()
            return
        if self.state == State.TRANSLATING:
            self.docking.ahead_entrance = list()
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
        self.state = State(self.state.value + 1)
        self.station_keeping_pub.publish(Empty())
    
    def approach(self):

        approach_point = self.docking.find_approach_point()

        if approach_point is None:
            return
        
        self.state = State(self.state.value + 1)
        
        self.trans_pub.publish(approach_point)
    
    def send_find_docks_req(self):
        
        self.get_logger().info('Sending Find Docks Request')
        self.state = State(self.state.value + 1)
        self.find_docks_req_pub.publish(Int8(data=1))
    
    def offsets_callback(self, msg:Int32MultiArray):
        if self.state != State.APPROACHING_COMPLETE:
            return
        self.docking.populate_color_docks(msg.data)

    
    def entrances_callback(self, msg:PointCloud2):
        self.docking.entrances = list()
        for point in read_points(msg):
            self.docking.entrances.append((point[0], point[1]))
    
    def ahead_entrance_callback(self, msg:PointCloud2):
        self.docking.ahead_entrance = list()
        for point in read_points(msg):
            self.docking.ahead_entrance.append((point[0], point[1]))
    
    def find_mid(self, p1, p2):
        return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    
    def translate_to_dock(self):
        translate_point = self.docking.find_y_translate_point()

        if translate_point is None:
            return
        
        self.state = State(self.state.value + 1)

        self.trans_pub.publish(translate_point)

    def trans_success_callback(self, msg):
        self.state = State(self.state.value + 1)
    
    def go_to_entrance(self):
        entrance_point = self.docking.find_entrance_point()

        if entrance_point is None:
            return

        self.state = State(self.state.value + 1)

        self.trans_pub.publish(entrance_point)
    
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