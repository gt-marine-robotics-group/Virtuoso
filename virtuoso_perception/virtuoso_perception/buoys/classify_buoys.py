import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ..utils.ColorFilter import ColorFilter
from ..utils.identify_buoys import find_buoys
from ..utils.classify_buoys import classify_buoys

class ClassifyBuoys(Node):

    def __init__(self):
        super().__init__('classify_buoys')
        self.boxes_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', self.record_boxes, 10)
        self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera/image_raw', self.record_image, 10)

        self.classified_buoys_pub = self.create_publisher(BoundingBoxArray, '/buoys/classified', 10)

        self.timer = self.create_timer(.1, self.classify_buoys)

        self.boxes = None
        self.image = None
    
    def record_boxes(self, msg:BoundingBoxArray):
        self.boxes = msg
    
    def record_image(self, msg:Image):
        self.image = msg
    
    def classify_buoys(self):

        if self.boxes is None or self.image is None: return

        bgr = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')

        buoys = find_buoys(bgr)

        # Sets the "value" property (defined by autoware.auto as any arbitrary float) of each box in
        # self.boxes to a number which represents the type of buoy it is
        classified_buoys = classify_buoys(self.boxes, buoys)

        bbArr = BoundingBoxArray()
        bbArr.boxes = classified_buoys
        self.classified_buoys_pub.publish(bbArr)


def main(args=None):
    
    rclpy.init(args=args)

    node = ClassifyBuoys()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
