import enum
import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import BoundingBoxArray
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time

class FindBuoys(Node):

    def __init__(self):
        super().__init__('find_buoys')
        self.boxes_sub = self.create_subscription(BoundingBoxArray, 'lidar_bounding_boxes', self.find_buoys, 10)
        self.boxes_pub = self.create_publisher(BoundingBoxArray, '/buoys/bounding_boxes', 10)

        self.buoy_counts = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def find_buoys(self, msg:BoundingBoxArray):

        filtered_boxes = BoundingBoxArray()

        filteredBoxesPrevFound = {}

        self.get_logger().info('msg ' + str(len(msg.boxes)))

        trans = None
        try:
            trans = self.tf_buffer.lookup_transform('map', 'wamv/lidar_wamv_link', Time()) 
        except:
            self.get_logger().info('TF BUFFER NOT WORKING')
            return
        
        for box in msg.boxes:
            for corner in box.corners:
                corner.x += trans.transform.translation.x
                corner.y += trans.transform.translation.y
            box.centroid.x += trans.transform.translation.x
            box.centroid.y += trans.transform.translation.y

        counter = 0
        for box in msg.boxes:
            # if box.centroid.z > .5: continue
            if math.sqrt((box.corners[1].x - box.corners[2].x)**2 + (box.corners[1].y - box.corners[2].y)**2) > 1: continue

            # need to update this check as now in map instead of lidar frame
            if math.sqrt(box.centroid.x**2 + box.centroid.y**2) > 20: continue

            if box.centroid.z + 1.55 > 0: 
                box.value = 1.0
            else:
                box.value = 0.5

            filtered_boxes.boxes.append(box)

            if not filteredBoxesPrevFound.get(counter):
                filteredBoxesPrevFound.update({counter: False})

            counter += 1

        self.get_logger().info('buoy_counts ' + str(len(self.buoy_counts)))
        self.get_logger().info('filtered_boxes ' + str(len(filtered_boxes.boxes)))


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
                    self.get_logger().info('found a repeat')
                    filteredBoxesPrevFound.update({i: True})
                    count.get('box').centroid.x = self.find_avg(prevBox.centroid.x, prevCount, box.centroid.x)
                    count.get('box').centroid.y = self.find_avg(prevBox.centroid.y, prevCount, box.centroid.y)
                    count.update({'count': prevCount + 10})
                    break

            if prevCount == count.get('count'):
                count.update({'count': prevCount - 1})
        
        self.get_logger().info('filteredBoxesPrevFound ' + str(filteredBoxesPrevFound))
            
        for key, prevFound in filteredBoxesPrevFound.items():
            if not prevFound:
                self.buoy_counts.append({
                    'box': filtered_boxes.boxes[key],
                    'count': 1
                })
        

        confirmedBuoys = BoundingBoxArray()

        confirmedBuoys.boxes = list(map(lambda b: b['box'], filter(lambda b: b['count'] > 90, self.buoy_counts)))

        self.get_logger().info('confirmedBuoys ' + str(len(confirmedBuoys.boxes)))
        
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
