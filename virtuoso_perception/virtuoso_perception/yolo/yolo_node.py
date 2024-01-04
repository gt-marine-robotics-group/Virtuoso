#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class YOLONode(Node):

    def __init__(self):
        super().__init__('YOLO')

        pkg_share = get_package_share_directory('virtuoso_perception')

        model_path = pkg_share + '/yolo/model.pt'

        self.cv_bridge = CvBridge()

        self.model = YOLO(model_path)

        self.threshold = 0.5

        self.image_sub = self.create_subscription(Image, 'input', 
            self.image_callback, 10)
        
        self.image_pub = self.create_publisher(Image, 'yolo_debug', 10)
    
    def image_callback(self, msg: Image):
        bgr = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(bgr)[0]

        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, conf, class_id = result
            
            if conf > self.threshold:
                conf_score = round(conf, 2)
                label = f"{results.names[int(class_id)]}: {conf_score}"
                cv2.rectangle(bgr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
                cv2.putText(bgr, label, (int(x1), int(y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3, cv2.LINE_AA)
        
        ros_image = self.cv_bridge.cv2_to_imgmsg(bgr, encoding='bgr8')

        self.image_pub.publish(ros_image) 


def main(args=None):
    rclpy.init(args=args)

    node = YOLONode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()