from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import List
import numpy as np
import cv2

class NoiseFilter:

    def __init__(self, denoising_params, image=None):

        self.denoising_params:List[int] = denoising_params
        self.image:Image = image

        self.cv_bridge = CvBridge()
    
    def filter(self):
        bgr:np.ndarray = self.cv_bridge.imgmsg_to_cv2(self.image, 'bgr8')

        filtered = cv2.fastNlMeansDenoisingColored(bgr, None, 
            *self.denoising_params
        )

        return self.cv_bridge.cv2_to_imgmsg(filtered, encoding='bgr8')
