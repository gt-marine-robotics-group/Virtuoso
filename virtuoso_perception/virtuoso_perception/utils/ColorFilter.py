import cv2
import numpy as np

class ColorFilter():

    def __init__(self, hsv, bgr):
        self.hsv = hsv
        self.bgr = bgr

    def black_filter(self, hsv_lower=[0,0,0], hsv_upper=[50,50,100]):
        lower = np.array(hsv_lower)
        upper = np.array(hsv_upper)

        return self.filter(lower, upper)

        # Could apply a bitwise_not to the mask to make the cone
        # actually appear black instead of white when returned.
        # However this would mean an extra case to consider when filtering for finding shapes
        # mask = cv2.inRange(self.hsv, lower, upper)

        # return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    def white_filter(self, hsv_lower=[0,0,0], hsv_upper=[0,0,255]):
        lower = np.array(hsv_lower)
        upper = np.array(hsv_upper)

        return self.filter(lower, upper)

    def red_orange_filter(self, hsv_lower1=[0,50,50], hsv_upper1=[10,255,255],
        hsv_lower2=[170,100,100], hsv_upper2=[180,255,255]):
        lower_red = np.array(hsv_lower1)
        upper_red = np.array(hsv_upper1)

        lower_red2 = np.array(hsv_lower2)
        upper_red2 = np.array(hsv_upper2)

        mask1 = cv2.inRange(self.hsv, lower_red, upper_red) 
        mask2 = cv2.inRange(self.hsv, lower_red2, upper_red2)

        filtered = cv2.bitwise_or(cv2.bitwise_and(self.bgr, self.bgr, mask=mask1), cv2.bitwise_and(self.bgr, self.bgr, mask=mask2))
        return filtered

    def green_filter(self, hsv_lower=[50,100,50], hsv_upper=[86,255,255]):
        lower_green = np.array(hsv_lower)
        upper_green = np.array(hsv_upper)

        return self.filter(lower_green, upper_green)
    
    def blue_filter(self, hsv_lower=[110,50,50], hsv_upper=[130,255,255]):
        lower_blue = np.array(hsv_lower)
        upper_blue = np.array(hsv_upper)

        return self.filter(lower_blue, upper_blue)
    
    def filter(self, lower, upper):

        mask = cv2.inRange(self.hsv, lower, upper)

        return cv2.bitwise_and(self.bgr, self.bgr, mask=mask)
