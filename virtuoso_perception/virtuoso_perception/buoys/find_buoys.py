import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import BoundingBoxArray
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from geometry_msgs.msg import Point32, PoseStamped
from ..utils.geometry_msgs import do_transform_pose_stamped

class FindBuoys(Node):

    def __init__(self):
        super().__init__('find_buoys')
        self.boxes_sub = self.create_subscription(BoundingBoxArray, 'lidar_bounding_boxes', self.find_buoys, 10)
        self.boxes_pub = self.create_publisher(BoundingBoxArray, '/buoys/bounding_boxes', 10)

        self.buoy_counts = []
    
    def to_pose_stamped(p:Point32):
        ps = PoseStamped()
        ps.header.frame_id = "wamv/lidar_wamv_link"
        ps.pose.position.x = p.x
        ps.pose.position.y = p.y
        ps.pose.position.z = p.z
        return ps

    def find_buoys(self, msg:BoundingBoxArray):

        filtered_boxes = BoundingBoxArray()

        filteredBoxesPrevFound = {}

        counter = 0
        for box in msg.boxes:
            if math.sqrt((box.corners[1].x - box.corners[2].x)**2 + (box.corners[1].y - box.corners[2].y)**2) > 1: continue

            if math.sqrt(box.centroid.x**2 + box.centroid.y**2) > 40: continue

            if box.centroid.z + 1.55 > 0: 
                box.value = 1.0
            else:
                box.value = 0.5

            filtered_boxes.boxes.append(box)

            if not filteredBoxesPrevFound.get(counter):
                filteredBoxesPrevFound.update({counter: False})

            counter += 1
        

        if len(self.buoy_counts) == 0:
            for i, _ in enumerate(filtered_boxes.boxes):
                filteredBoxesPrevFound.update({i: False})

        for count in self.buoy_counts:
            prevBox = count.get('box')
            prevCount = count.get('count')

            for i, box in enumerate(filtered_boxes.boxes):
                if (filteredBoxesPrevFound.get(i)):
                    continue
                if math.sqrt((prevBox.centroid.x - box.centroid.x)**2 + (prevBox.centroid.y - box.centroid.y)**2) < 3:
                    filteredBoxesPrevFound.update({i: True})
                    count.get('box').centroid.x = self.find_avg(prevBox.centroid.x, prevCount, box.centroid.x)
                    count.get('box').centroid.y = self.find_avg(prevBox.centroid.y, prevCount, box.centroid.y)
                    count.update({'count': prevCount + 10})
                    break

            if prevCount == count.get('count') and prevCount > 0:
                count.update({'count': prevCount - 1})
        
        for key, prevFound in filteredBoxesPrevFound.items():
            if not prevFound:
                self.buoy_counts.append({
                    'box': filtered_boxes.boxes[key],
                    'count': 1
                })
        

        confirmedBuoys = BoundingBoxArray()

        confirmedBuoys.boxes = list(map(lambda b: b['box'], filter(lambda b: b['count'] > 90, self.buoy_counts)))

        self.boxes_pub.publish(confirmedBuoys)

    
    def find_avg(self, curr:float, count:int, next:float):
        sum = (curr * count) + next
        return sum / (count + 1)


def main(args=None):
    
    rclpy.init(args=args)

    node = FindBuoys()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
