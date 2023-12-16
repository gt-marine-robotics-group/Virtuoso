from virtuoso_navigation.planners.Planner import Planner
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
import math

class StraightPath(Planner):

    def create_path(self, goal:Pose) -> Path:
        if goal.position.x - self.robot_pose.position.x > 0:
            x_dir = 1
        else:
            x_dir = -1
        
        if goal.position.y - self.robot_pose.position.y > 0:
            y_dir = 1
        else:
            y_dir = -1

        path = Path()
        path.header.frame_id = 'map'

        if goal.position.x - self.robot_pose.position.x == 0.0:
            theta = .5 * math.pi
        else:
            theta = math.atan(abs(
                (goal.position.y - self.robot_pose.position.y) / (goal.position.x - self.robot_pose.position.x)
            ))

        curr_pose = Planner.pose_deep_copy(self.robot_pose)

        while (
            (abs(curr_pose.position.x - self.robot_pose.position.x) 
                < abs(goal.position.x - self.robot_pose.position.x)) and 
            (abs(curr_pose.position.y - self.robot_pose.position.y) 
                < abs(goal.position.y - self.robot_pose.position.y))
        ):
            path.poses.append(PoseStamped(pose=Planner.pose_deep_copy(curr_pose)))

            curr_pose.position.x += math.cos(theta) * 0.2 * x_dir
            curr_pose.position.y += math.sin(theta) * 0.2 * y_dir

        path.poses.append(PoseStamped(pose=goal))

        return path
