from abc import ABC, abstractmethod
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose
from virtuoso_mapping.inflation_layer import InflationLayer

class Planner(ABC):

    def __init__(self):

        self.map: OccupancyGrid = None
        self.robot_pose: Pose = None

        # self._inflation_layer = InflationLayer(inflation_layer)

        self.node = None
    
    def debug(self, msg: str):
        if self.node is None:
            return
        self.node.get_logger().info(msg)
    
    @abstractmethod
    def create_path(self, goal: Pose) -> Path:
        pass

    def pose_deep_copy(pose:Pose):
        copy = Pose()
        copy.position.x = pose.position.x
        copy.position.y = pose.position.y
        copy.position.z = pose.position.z
        copy.orientation.x = pose.orientation.x
        copy.orientation.y = pose.orientation.y
        copy.orientation.z = pose.orientation.z
        copy.orientation.w = pose.orientation.w

        return copy