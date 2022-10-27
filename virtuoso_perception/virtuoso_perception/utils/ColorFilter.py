import cv2
import numpy as np

class ColorFilter():

    def __init__(self, hsv, bgr):
        self.hsv = hsv
        self.bgr = bgr

    def black_filter(self):
        lower = np.array([0, 0, 0])
        upper = np.array([50, 50, 100])

        # Could apply a bitwise_not to the mask to make the cone
        # actually appear black instead of white when returned.
        # However this would mean an extra case to consider when filtering for finding shapes
        mask = cv2.inRange(self.hsv, lower, upper)

        return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    def white_filter(self):
        lower = np.array([0, 0, 0])
        upper = np.array([0, 0, 255])

        return self.filter(lower, upper)

    def red_orange_filter(self, white_filtered):
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        # return self.filter(lower_red, upper_red)

        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # return self.filter(lower_red2, upper_red2)

        mask1 = cv2.inRange(self.hsv, lower_red, upper_red) 
        mask2 = cv2.inRange(self.hsv, lower_red2, upper_red2)

        filtered = cv2.bitwise_or(cv2.bitwise_and(self.bgr, self.bgr, mask=mask1), cv2.bitwise_and(self.bgr, self.bgr, mask=mask2))
        return filtered

        # return cv2.bitwise_xor(filtered, white_filtered)

    def green_filter(self):
        lower_green = np.array([50, 100, 50])
        upper_green = np.array([86, 255, 255])

        return self.filter(lower_green, upper_green)
    
    def blue_filter(self):
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])

        return self.filter(lower_blue, upper_blue)
    
    def filter(self, lower, upper):

        mask = cv2.inRange(self.hsv, lower, upper)

        return cv2.bitwise_and(self.bgr, self.bgr, mask=mask)
