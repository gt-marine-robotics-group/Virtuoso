from geometry_msgs.msg import Point32

# Values Guide:
# 0 = tall black = mb_marker_buoy_black
# 1 = tall green = mb_marker_buoy_green
# 2 = tall red = mb_marker_buoy_red
# 3 = tall white = mb_marker_buoy_white
# 4 = round black = mb_round_buoy_black
# 5 = round orange = mb_round_buoy_orange

def determine_type(num:int):
    if (num == 0): return 'Tall-Black', 'mb_marker_buoy_black'
    if (num == 1): return 'Tall-Green', 'mb_marker_buoy_green' 
    if (num == 2): return 'Tall-Red', 'mb_marker_buoy_red'
    if (num == 3): return 'Tall-White', 'mb_marker_buoy_white'
    if (num == 4): return 'Round-Black', 'mb_round_buoy_black'
    return 'Round-Orange', 'mb_round_buoy_orange' 

class Buoy():

    def __init__(self, buoy:int):
        self.num = buoy.value
        self.name, self.code = determine_type(buoy.value)
        self.centroid:Point32 = buoy.centroid

        self.geo_msg = None