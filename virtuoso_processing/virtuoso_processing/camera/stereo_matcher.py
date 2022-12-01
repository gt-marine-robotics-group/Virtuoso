import cv2
import numpy as np
import matplotlib.pyplot as plt
import open3d
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
            numDisparities=num_disp,
            blockSize=11,
        )
    
    def match(self, img1, img2):
        disparity = self.matcher.compute(img1, img2).astype(np.float32) / 16.0
        return disparity
    
    def reconstruct(self, img_disp0, img_rect0, Q):
        self.node.get_logger().info(str(np.shape(img_rect0)))
        # reproject disparity to 3D
        xyz = cv2.reprojectImageTo3D(img_disp0, Q)

        # construct validity masks based on distance and brightness
        mask_depth = (xyz[:,:,2] < 5.0) & (xyz[:,:,2] > 0.1)
        self.node.get_logger().info(f'mask_depth: {mask_depth}')
        self.node.get_logger().info(f'has true: {True in mask_depth}')
        mask_bright = (img_rect0 > 30) & (img_rect0 < 250)

        # self.node.get_logger().info(str(np.shape(mask_bright)))

        # create linear point and color lists
        xyz_linear = xyz.reshape((-1, 3))
        colors_linear = img_rect0.reshape((-1, 3))
        mask_linear = (mask_bright[:,:,0] & mask_depth).flatten()

        # create open3d geometry
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(xyz_linear[mask_linear])
        pcd.colors = open3d.utility.Vector3dVector(colors_linear[mask_linear] / 255.0)

        return pcd