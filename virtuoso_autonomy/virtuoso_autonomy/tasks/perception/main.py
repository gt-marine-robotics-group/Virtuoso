import rclpy
from rclpy.node import Node
from virtuoso_msgs.msg import Task
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from typing import List
from .utils.Buoy import Buoy
from geographic_msgs.msg import GeoPoseStamped, GeoPose, GeoPoint
from geometry_msgs.msg import Quaternion

class Perception(Node):

    def __init__(self):
        super().__init__('perception')

        self.task_info_sub = self.create_subscription(Task, '/vrx/task/info', self.task_info_callback, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, 'buoys/classified', self.buoys_callback, 10)

        self.response_pub = self.create_publisher(GeoPoseStamped, '/vrx/perception/landmark', 10)

        self.is_task3 = False 
        self.state = 'initial'
        self.prev_buoys:List[BoundingBoxArray] = []
        self.response_sent = False
    
    def task_info_callback(self, msg:Task):

        if (msg.name == 'perception'): 
            self.update_state(msg)

    def update_state(self, msg:Task):
        self.state = msg.state
    
    def buoys_callback(self, msg:BoundingBoxArray):

        if (self.state != 'running'): return
        
        self.add_to_prev_buoys(msg)

        buoys, none_identified = self.find_most_identified_buoys()

        if (none_identified):
            self.response_sent = False
            return
        
        if (self.response_sent or len(buoys) == 0): return
        
        for buoy in buoys:
            # convert the buoy centroid into latitude and longitude
            # send data to the response_pub
            self.get_logger().info(str(buoy.name))
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