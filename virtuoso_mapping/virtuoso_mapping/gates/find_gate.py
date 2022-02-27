from numpy import Infinity
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from autoware_auto_perception_msgs.msg import BoundingBoxArray, BoundingBox
from tf2_ros import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Vector3
import math
from typing import List

def same_location(a:BoundingBox, b:BoundingBox):
    x_diff = a.centroid.x - b.centroid.x 
    y_diff = a.centroid.y - b.centroid.y
    if (x_diff < -1 or x_diff > 1): return False
    if (y_diff < -1 or y_diff > 1): return False
    return True

def calc_dist(buoy:BoundingBox, currentPos:Vector3):
    return math.sqrt((buoy.centroid.x - currentPos.x)**2 + (buoy.centroid.y - currentPos.y)**2)

class FindGate(Node):

    def __init__(self):
        super().__init__('find_gate')
        
        self.buoys_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', self.find_gate, 10)

        self.gate_pub = self.create_publisher(BoundingBoxArray, '/buoys/gate', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.passed_buoys = BoundingBoxArray()
        self.closest_buoys:List[List[BoundingBox]] = []
        self.gate_identified = BoundingBoxArray()
    
    def find_gate(self, msg:BoundingBoxArray):

        lidar_to_odom:TransformStamped = None

        try:
            lidar_to_odom = self.tf_buffer.lookup_transform('odom', 'wamv/lidar_wamv_link', Time())
        except:
            return

        for buoy in msg.boxes:
            buoy.centroid.x += lidar_to_odom.transform.translation.x
            buoy.centroid.y += lidar_to_odom.transform.translation.y

        two_closest_buoys = self.find_2_closest_new_buoys(msg, lidar_to_odom) 

        self.closest_buoys.insert(0, two_closest_buoys)

        if (len(self.closest_buoys) > 10): self.closest_buoys.pop()
        else: return

        gate_buoys = []

        for c_buoys in self.closest_buoys:
            for buoy in c_buoys:
                if (buoy is None): continue
                added = False
                for gate_buoy in gate_buoys:
                    if (same_location(buoy, gate_buoy['box'])):
                        gate_buoy['box'].centroid.x = ((gate_buoy['box'].centroid.x * gate_buoy['count']) + buoy.centroid.x) / (gate_buoy['count'] + 1)
                        gate_buoy['box'].centroid.y = ((gate_buoy['box'].centroid.y * gate_buoy['count']) + buoy.centroid.y) / (gate_buoy['count'] + 1)
                        gate_buoy['count'] += 1
                        added = True
                        break
                if (not added):
                    gate_buoys.append({'box': buoy, 'count': 1})
        
        # As long as one of the buoys has count > 8, we will just assume
        # the second highest count is the other gate buoy

        if (len(gate_buoys) < 2): return

        gate_buoys.sort(key=lambda b: b['count'], reverse=True)

        if (gate_buoys[0]['count'] < 8): return

        # either publish a new gate_identified if none previously
        if (len(self.gate_identified.boxes) == 0):
            # self.get_logger().info(str('MAKING NEW GATE'))
            self.gate_identified.boxes = [gate_buoys[0]['box'], gate_buoys[1]['box']]
            self.gate_pub.publish(self.gate_identified)
            return

        # validate the old gate_identified and modify if necessary
        # self.get_logger().info(str('VALIDATING OLD GATE'))
        for buoy in gate_buoys:
            if (same_location(buoy['box'], self.gate_identified.boxes[0]) or same_location(buoy['box'], self.gate_identified.boxes[1])): continue
            # self.get_logger().info(str('MAKING NEW GATE'))
            self.gate_identified.boxes = [gate_buoys[0]['box'], gate_buoys[1]['box']]
            self.gate_pub.publish(self.gate_identified)
            return
    
    def find_2_closest_new_buoys(self, buoys:BoundingBoxArray, trans:TransformStamped):
        closest_buoys = [None, None]
        closest_distances = [math.inf, math.inf]

        for buoy in buoys.boxes:

            already_passed = False
            for passed_buoys in self.passed_buoys.boxes:
                if (same_location(passed_buoys, buoy)): 
                    already_passed = True
                    break
            if (already_passed): continue

            distance = calc_dist(buoy, trans.transform.translation)
            
            for i, c_dist in enumerate(closest_distances):
                if (c_dist is None or distance < c_dist):
                    closest_distances.insert(i, distance)
                    closest_distances.pop()
                    closest_buoys.insert(i, buoy)
                    closest_buoys.pop()
                    break
        
        return closest_buoys
            



def main(args=None):
    
    rclpy.init(args=args)

    node = FindGate()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()