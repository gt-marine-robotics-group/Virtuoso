import cv2
import numpy as np

def find_contours(bgr):
    canny = cv2.Canny(bgr, 50, 150)

    kernel = np.ones((3))

    dilated = cv2.dilate(canny, kernel, iterations=1)

    contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours