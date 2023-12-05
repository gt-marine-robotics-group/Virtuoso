from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from virtuoso_navigation.planners.Planner import Planner

class RRT(Planner):

    def create_path(self, goal: Pose) -> Path:
        pass 