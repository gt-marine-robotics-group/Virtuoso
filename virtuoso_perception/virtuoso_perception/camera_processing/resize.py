from sensor_msgs.msg import CameraInfo, Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class Resize:

    def __init__(self, resize_factor, image=None, camera_info=None):

        self.resize_factor:int = resize_factor

        self.image:Image = image
        self.camera_info:CameraInfo = camera_info

        self.cv_bridge = CvBridge()
    
    def resize(self):
        bgr:np.ndarray = self.cv_bridge.imgmsg_to_cv2(self.image, 'bgr8')

        resize = cv2.resize(bgr, (bgr.shape[1] // self.resize_factor, 
            bgr.shape[0] // self.resize_factor), 
            interpolation=cv2.INTER_AREA
        )

        info = self.camera_info
        info.k[0] /= self.resize_factor
        info.k[2] /= self.resize_factor
        info.k[4] /= self.resize_factor
        info.k[5] /= self.resize_factor
        info.width //= self.resize_factor
        info.height //= self.resize_factor
        
        return self.cv_bridge.cv2_to_imgmsg(resize, encoding='bgr8'), info