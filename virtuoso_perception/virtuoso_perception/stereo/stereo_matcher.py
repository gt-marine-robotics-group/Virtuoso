import cv2
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node

class StereoMatcherSGBM(cv2.StereoSGBM):

    def __init__(self, node):

        self.node:Node = node

        window_size = 11
        min_disp = 0
        num_disp = 320 - min_disp

        # self.matcher = cv2.StereoSGBM_create(
        #     minDisparity=min_disp,
        #     numDisparities=num_disp,
        #     uniquenessRatio=5,
        #     speckleRange=5, # docs suggest 1-2 should be good enough
        #     disp12MaxDiff=1,
        #     P1=8*2*window_size**2,
        #     P2=32*2*window_size**2
        # )
        self.matcher = cv2.StereoBM_create(
            numDisparities=64,
            blockSize=17
        )
        self.matcher.setSpeckleWindowSize(10)
    
    def match(self, img1, img2):
        disparity = self.matcher.compute(img1, img2).astype(np.float32) / 16.0
        return disparity