import rclpy
from rclpy.node import Node
from virtuoso_msgs.msg import Task
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from typing import List
from .utils.Buoy import Buoy
from .utils.transform_buoys import transform_buoys
from geographic_msgs.msg import GeoPoseStamped, GeoPose, GeoPoint
from sensor_msgs.msg import NavSatFix
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time

class Perception(Node):

    def __init__(self):
        super().__init__('perception')

        self.task_info_sub = self.create_subscription(Task, '/vrx/task/info', self.task_info_callback, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, 'buoys/classified', self.buoys_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/filtered', self.gps_callback, 10)

        self.response_pub = self.create_publisher(GeoPoseStamped, '/vrx/perception/landmark', 10)

        self.is_task3 = False 
        self.state = 'initial'
        self.prev_buoys:List[BoundingBoxArray] = []
        self.response_sent = False
        self.gps_pos = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def gps_callback(self, msg:NavSatFix):
        self.gps_pos = msg

    def task_info_callback(self, msg:Task):

        if (msg.name == 'perception'): 
            self.update_state(msg)

    def update_state(self, msg:Task):
        self.state = msg.state
    
    def buoys_callback(self, msg:BoundingBoxArray):

        if (self.state != 'running'): return
        if (self.gps_pos is None): return
        
        self.add_to_prev_buoys(msg)

        buoys, none_identified = self.find_most_identified_buoys()

        if (none_identified):
            self.response_sent = False
            return
        
        if (self.response_sent or len(buoys) == 0): return

        # translate all the buoys to have a lat and lon
        trans_buoys = transform_buoys(buoys, self.tf_buffer, self.gps_pos) 

        if (trans_buoys is None):
            self.get_logger().info('Failed to transform between utf and lidar frames')
            return
        
        for buoy in buoys:
            # Publish the Geo message to get scored!
            self.response_pub.publish(buoy.geo_msg)
        self.response_sent = True

    def add_to_prev_buoys(self, msg:BoundingBoxArray):

        if (len(self.prev_buoys) == 10): self.prev_buoys.pop(0)
        self.prev_buoys.append(msg)
    
    def none_identified(self, buoysList:List[BoundingBoxArray]):
        for buoys in buoysList:
            if (len(buoys) > 0): return False
        return True
    
    def find_most_identified_buoys(self):

        if (len(self.prev_buoys) < 8): return [], True

        counts = [[], [], [], [], [], []]

        for buoys in self.prev_buoys:
            for buoy in buoys.boxes:
                counts[int(buoy.value)].append(buoy)
        
        most_identified:List[Buoy] = []

        for i, buoys in enumerate(counts):
            if (len(buoys) > 8): 
                most_identified.append(Buoy(buoys[0]))
                continue
            
            # Sometimes the second buoy is spawned slightly after the first,
            # so we need to give it more time to be detected.
            if (len(buoys) > 2):
                return [], False 
        
        return most_identified, self.none_identified(counts)

def main(args=None):
    
    rclpy.init(args=args)

    node = Perception()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()