from geometry_msgs.msg import TransformStamped
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from virtuoso_msgs.msg import BuoyArray
from virtuoso_msgs.srv import Channel

class FindChannel: 
    
    def __init__(self):

        self.cam_to_map_trans:TransformStamped = None
        
        self.lidar_buoys:BoundingBoxArray = None
        self.camera_buoys:BuoyArray = None

        # When count becomes greater than some number, just fall back to 
        # using LIDAR only if that is an option
        self.iteration_count = 0
    
    def reset(self):
        self.lidar_buoys = None
        self.camera_buoys = None
        self.iteration_count = 0
    
    def execute(self, res:Channel.Response):
        pass 
    
    def _find_cam_buoys(self):
        pass