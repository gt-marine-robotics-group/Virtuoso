from typing import List, Tuple
from geometry_msgs.msg import PoseStamped, Pose
import math
import tf_transformations

class ChannelNavigation:

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

        buoys.sort(key=lambda b: ChannelNavigation._distance(loc, b))

        filteredBuoys = list(filter(lambda b: not self._isPrevNavigatedBuoy(b), buoys))

        if len(filteredBuoys) < 2:
            return self.curr_channel

        self.curr_channel = (filteredBuoys[0], filteredBuoys[1])

        return self.curr_channel

    def _isPrevNavigatedBuoy(self, buoy:PoseStamped):
        
        for channel in self.channels:
            for channelBuoy in channel:
                if ChannelNavigation._distance(channelBuoy, buoy) < self.buoy_err_range:
                    return True
        
        return False
    
    def _distance(p1:PoseStamped, p2:PoseStamped):
        return math.sqrt((p1.pose.position.x - p2.pose.position.x)**2 + (p1.pose.position.y - p2.pose.position.y)**2)
    
    def find_midpoint(p1:PoseStamped, p2:PoseStamped, loc:PoseStamped):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = (p1.pose.position.x + p2.pose.position.x) / 2
        ps.pose.position.y = (p1.pose.position.y + p2.pose.position.y) / 2

        ang = math.atan2((p1.pose.position.y - p2.pose.position.y), (p1.pose.position.x - p2.pose.position.x)) - (math.pi / 2)

        while ang < 0:
            ang += (2 * math.pi)

        rq = loc.pose.orientation
        robot_euler = tf_transformations.euler_from_quaternion([rq.x, rq.y, rq.z, rq.w])

        robot_yaw = robot_euler[2]
        if robot_yaw < 0:
            robot_yaw += (math.pi * 2)

        if ang > math.pi * 2:
            ang = ang % (math.pi * 2)

        if abs(ang - robot_yaw) > abs(((ang + math.pi) % (math.pi * 2)) - robot_yaw) and math.pi*2 - abs(ang - robot_yaw) > abs(((ang + math.pi) % (math.pi * 2)) - robot_yaw):
            ang += math.pi
        
        quat = tf_transformations.quaternion_from_euler(0, 0, ang)
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        return ps

