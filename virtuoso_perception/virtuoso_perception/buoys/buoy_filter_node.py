import rclpy
from rclpy.node import Node
from virtuoso_perception.utils.ColorFilter import ColorFilter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ..utils.code_identification import find_contours
import numpy as np
from scipy import stats

class BuoyColorFilterNode(Node):

    def __init__(self):
        super().__init__('perception_buoy_color_filter')

        self.declare_parameters(namespace='', parameters=[
            ('topic', ''),
        ])

        base_topic = self.get_parameter('topic').value

        self.image_sub = self.create_subscription(Image, 
            f'{base_topic}/image_raw', self.image_callback, 10)

        self.filter_pub = self.create_publisher(Image,
            f'{base_topic}/buoy_filter', 10)

        self.black_white_debug_pub = self.create_publisher(Image,
            f'{base_topic}/buoy_filter/debug/black_white', 10)
        self.full_contours_debug_pub = self.create_publisher(Image,
            f'{base_topic}/buoy_filter/debug/full_contours', 10)
        self.filtered_contours_debug_pub = self.create_publisher(Image,
            f'{base_topic}/buoy_filter/debug/filtered_contours', 10)
    
    def contour_filter(self, bgr_img:np.ndarray):

        img_shape = np.shape(bgr_img)

        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        _, black_and_white = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
        contours = find_contours(black_and_white)

        filtered_img = np.zeros(np.shape(black_and_white)).astype('uint8')

        self.black_white_debug_pub.publish(CvBridge().cv2_to_imgmsg(black_and_white, encoding='mono8'))
        self.full_contours_debug_pub.publish(
            CvBridge().cv2_to_imgmsg(cv2.drawContours(
                    bgr_img.copy(), contours, -1, (255,0,0), 1
                ),
                encoding='bgr8')
        )

        filtered = list()
        for i, cnt in enumerate(contours):

            area = cv2.contourArea(cnt)
            if area < 100:
                continue

            # create image with specific contour filled in
            blank = np.zeros((img_shape[0], img_shape[1]))
            filled = cv2.drawContours(blank, contours, i, 255, -1)
            
            # get the index of all pixels within the contour
            pts = np.where(filled == 255)

            x_to_y = dict()
            y_to_x = dict()

            for pt in cnt:
                pt = pt[0]
                if pt[0] in x_to_y:
                    x_to_y[pt[0]].append(pt[1])
                else:
                    x_to_y[pt[0]] = [pt[1]]
                
                if pt[1] in y_to_x:
                    y_to_x[pt[1]].append(pt[0])
                else:
                    y_to_x[pt[1]] = [pt[0]]

            # get the hue of the hsv_img at each index
            color = list()
            for i in range(pts[0].size):
                on_border = False

                if pts[0][i] in x_to_y:
                    for pt in x_to_y[pts[0][i]]:
                        if abs(pt - pts[1][i]) < 5:
                            on_border = True
                            break
                if on_border: continue

                if pts[1][i] in y_to_x:
                    for pt in y_to_x[pts[1][i]]:
                        if abs(pt - pts[0][i]) < 5:
                            on_border = True
                            break
                if on_border: continue

                color.append(black_and_white[pts[0][i]][pts[1][i]])

            mode = stats.mode(color)

            if mode.mode[0] != 255:
                continue
            
            if mode.count[0] / len(color) < .70:
                continue
            
            for i in range(pts[0].size):
                filtered_img[pts[0][i]][pts[1][i]] = 255
            
            filtered.append(cnt)
        
        drawn = cv2.drawContours(bgr_img.copy(), filtered, -1, (255,0,0), 3)
        self.filtered_contours_debug_pub.publish(CvBridge().cv2_to_imgmsg(drawn, encoding='bgr8'))

        return filtered_img
    
    def apply_filter(self, img:Image):
        bgr_image = CvBridge().imgmsg_to_cv2(img, desired_encoding='bgr8')

        self.color_filter = ColorFilter(cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV), bgr_image)

        red_filtered = self.color_filter.red_orange_filter(hsv_lower1=[0,50,50], 
            hsv_upper1=[10,255,255], hsv_lower2=[160,50,50], hsv_upper2=[180,255,255])
        green_filtered = self.color_filter.green_filter(hsv_lower=[50,50,20])
        black_filtered = self.color_filter.black_filter()
        yellow_filtered = self.color_filter.yellow_filter()

        combo = cv2.bitwise_or(
            cv2.bitwise_or(cv2.bitwise_or(red_filtered, green_filtered), yellow_filtered),
            black_filtered 
        )

        contour_filtered = self.contour_filter(combo)

        return CvBridge().cv2_to_imgmsg(contour_filtered, encoding='mono8')
    
    def image_callback(self, msg:Image):
        self.filter_pub.publish(self.apply_filter(msg))


def main(args=None):
    
    rclpy.init(args=args)

    sub = BuoyColorFilterNode()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()