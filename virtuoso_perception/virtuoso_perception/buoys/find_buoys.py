import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import BoundingBoxArray
import math

class FindBuoys(Node):

    def __init__(self):
        super().__init__('find_buoys')
        self.boxes_sub = self.create_subscription(BoundingBoxArray, 'lidar_bounding_boxes', self.find_buoys, 10)
        self.boxes_pub = self.create_publisher(BoundingBoxArray, '/buoys/bounding_boxes', 10)

    def find_buoys(self, msg:BoundingBoxArray):

        filtered_boxes = BoundingBoxArray()

        for box in msg.boxes:
            if box.corners[1].x < -5 or box.corners[1].y < -5 or box.corners[1].y > 20: continue
            if box.centroid.z > .5: continue
            if math.sqrt((box.corners[1].x - box.corners[2].x)**2 + (box.corners[1].y - box.corners[2].y)**2) > 1: continue

            if box.centroid.z + 1.8 > .2: 
                box.value = 1.0
            else:
                box.value = 0.5
            filtered_boxes.boxes.append(box)
        
        self.boxes_pub.publish(filtered_boxes)


def main(args=None):
    
    rclpy.init(args=args)

    node = FindBuoys()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
