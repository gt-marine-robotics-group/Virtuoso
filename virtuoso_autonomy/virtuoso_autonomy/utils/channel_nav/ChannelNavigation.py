from typing import List, Tuple
from geometry_msgs.msg import PoseStamped, Pose
import math

class ChannelNavigation():

    channels:List[Tuple[PoseStamped, PoseStamped]] = []
    curr_channel:Tuple[PoseStamped, PoseStamped] = None
    end_nav:bool = False
    buoy_err_range = 3.0

    # Assume positions coming in are in the map/odom frame
    def find_channel(self, buoys:List[PoseStamped], loc:PoseStamped):

        if self.end_nav:
            return

        if self.curr_channel:
            self.channels.append(self.curr_channel)
            self.curr_channel = None

        buoys.sort(key=lambda b: ChannelNavigation.distance(loc, b))

        filteredBuoys = list(filter(lambda b: not self.isPrevNavigatedBuoy(b), buoys))

        if len(filteredBuoys) < 2:
            return self.curr_channel

        self.curr_channel = (filteredBuoys[0], filteredBuoys[1])

        return self.curr_channel

    def isPrevNavigatedBuoy(self, buoy:PoseStamped):
        
        for channel in self.channels:
            for channelBuoy in channel:
                if ChannelNavigation.distance(channelBuoy, buoy) < self.buoy_err_range:
                    return True
        
        return False
    
    def distance(p1:PoseStamped, p2:PoseStamped):
        return math.sqrt((p1.pose.position.x - p2.pose.position.x)**2 + (p1.pose.position.y - p2.pose.position.y)**2)

