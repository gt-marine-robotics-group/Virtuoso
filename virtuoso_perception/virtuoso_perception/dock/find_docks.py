from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from rclpy.node import Node

class FindDocks:

    def __init__(self):

        self.image:Image = None
        # self.search_requested:bool = False
        self.search_requested:bool = True
    
    def get_ready_msg(self):
        if self.search_requested:
            return None
        msg = Int32() 
        msg.data = 1
        return msg
    
    def find(self, node:Node):

        if not self.search_requested:
            return
        
        if self.image is None:
            return
        

        bgr = CvBridge().imgmsg_to_cv2(self.image, desired_encoding='bgr8')

        shape = bgr.shape

        node.get_logger().info(f'Shape: {shape}')