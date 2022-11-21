import rclpy
from rclpy.node import Node
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ..utils.ColorFilter import ColorFilter

class DetectedBuoy:

    def __init__(self, color, x, y, w, h):
        self.color = color
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.area = w * h

class ClassifyBuoys(Node):

    def __init__(self):
        super().__init__('perception_classify_buoys')
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

        buoys = self.find_buoys(bgr)

        # Sets the "value" property (defined by autoware.auto as any arbitrary float) of each box in
        # self.boxes to a number which represents the type of buoy it is
        classified_buoys = self.classify_buoys(self.boxes, buoys)

        bbArr = BoundingBoxArray()
        bbArr.boxes = classified_buoys
        self.classified_buoys_pub.publish(bbArr)

    def classify_buoys2(self, boxes, buoys):

        sorted_boxes = self.sort_boxes(boxes)
        sorted_buoys = self.sort_buoys(buoys)

        for i, box in enumerate(sorted_boxes):

            if i >= len(sorted_buoys):
                # just guess as camera did not identify a buoy
                box.value = 0.0 if box.value == 1.0 else 4.0
                continue
                
            buoy = sorted_buoys[i]

            if buoy.color == 'black':
                box.value = 0.0 if box.value == 1.0 else 4.0
                continue

            if buoy.color == 'red_or_orange':
                box.value = 2.0 if box.value == 1.0 else 5.0
                continue

            if buoy.color == 'green':
                box.value = 1.0
                continue

            box.value = 3.0
        
        return sorted_boxes
        
    def sort_buoys(self, buoys):

        sorted_arr = []

        for buoy in buoys:
            inserted = False

            for i, sorted_buoy in enumerate(sorted_arr):
                if sorted_buoy.x > buoy.x:
                    sorted_arr.insert(i, buoy)
                    inserted = True
                    break

            if not inserted: sorted_arr.append(buoy)
        
        return sorted_arr

    def sort_boxes(self, boxes:BoundingBoxArray):
        
        sorted_arr = []

        for box in boxes.boxes:
            inserted = False 

            for i, sorted_box in enumerate(sorted_arr):
                if sorted_box.centroid.y < box.centroid.y:
                    sorted_arr.insert(i, box)
                    inserted = True
                    break

            if not inserted: sorted_arr.append(box)
        
        return sorted_arr

    def find_buoys_for_color(self, bgr, color):

        canny = cv2.Canny(bgr, 50, 150)

        kernel = np.ones((3))

        dilated = cv2.dilate(canny, kernel, iterations=1)

        contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        buoys = []

        for cnt in contours:

            area = cv2.contourArea(cnt)

            if area < 1000: continue 

            x, y, w, h = cv2.boundingRect(cnt)

            buoys.append(DetectedBuoy(color, x, y, w, h))
        
        return buoys

    def find_buoys(self, bgr):

        hsv:np.ndarray = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        filter = ColorFilter(hsv, bgr)

        white = filter.white_filter()
        black = filter.black_filter()
        red_or_orange = filter.red_orange_filter()
        green = filter.green_filter()

        white_buoys = self.find_buoys_for_color(white, 'white')
        black_buoys = self.find_buoys_for_color(black, 'black')
        red_or_orange_buoys = self.find_buoys_for_color(red_or_orange, 'red_or_orange')
        green_buoys = self.find_buoys_for_color(green, 'green')

        buoys = white_buoys + black_buoys + red_or_orange_buoys + green_buoys

        return buoys


def main(args=None):
    
    rclpy.init(args=args)

    node = ClassifyBuoys()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
