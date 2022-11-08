import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
from .wildlife_encounter_states import State
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from math import sqrt, pi
import tf_transformations
from ...utils.multi_gates.multi_gates import MultiGates
from ...utils.multi_gates.loop_point import LoopPoint
import time

class WildlifeEncounterNode(Node):

    possible_animals = ['Platypus', 'Crocodile', 'Turtle']

    def __init__(self):
        super().__init__('autonomy_wildlife_encounter')

        self.path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)
        self.trans_pub = self.create_publisher(Point, '/navigation/translate', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/virtuoso_navigation/success',
            self.nav_success_callback, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', 
            self.buoys_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)
        
        self.state = State.SEARCHING

        self.robot_pose:Pose = None
        self.animals = dict()
        self.first_call = True

        self.create_timer(1.0, self.execute)
    
    def execute(self):
        if self.first_call:
            self.first_call = False
            time.sleep(3.0)
        self.get_logger().info(f'state: {self.state}')
        self.get_logger().info(str(list(x[1] for x in self.animals.values())))
        if self.state == State.SEARCHING:
            if not self.search():
                self.nav_search()
            return
        if self.state == State.SEARCH_TRANSLATING:
            return
        if self.state == State.SEARCH_TRANSLATING_DONE:
            if not self.search():
                self.rotate()
            return
        if self.state == State.SEARCH_ROTATING1:
            return
        if self.state == State.SEARCH_ROTATING1_DONE:
            if not self.search():
                self.rotate()
            return
        if self.state == State.SEARCH_ROTATING2:
            return
        if self.state == State.SEARCH_ROTATING2_DONE:
            if not self.search():
                self.state = State.SEARCHING
            return
        if self.state == State.ANIMAL_NAVIGATING:
            return
        self.get_logger().info('COMPLETE') 
    
    def search(self):
        name = self.find_animal_to_navigate() 
        if not name is None:
            self.nav_animal(name)
            return True
        return False
            
    def find_animal_to_navigate(self):
        for name, data in self.animals.items():
            if not data[1]:
                return name
        return None
    
    def nav_search(self):
        self.state = State.SEARCH_TRANSLATING
        self.rotations_done = 0
        self.trans_pub.publish(Point(x=10.0))

    def odom_callback(self, msg:Odometry):
        self.robot_pose = msg.pose.pose
    
    def buoys_callback(self, msg:BoundingBoxArray):
        for buoy in msg.boxes:
            if len(self.animals) > 2: return
            for animal, data in self.animals.items():
                if self.distance(data[0], (buoy.centroid.x, buoy.centroid.y)) > 3:
                    break
            else:
                index = WildlifeEncounterNode.possible_animals[len(self.animals) - 1]
                self.animals[index] = [(buoy.centroid.x, buoy.centroid.y), False]
    
    def nav_success_callback(self, msg):
        if (self.state == State.ANIMAL_NAVIGATING and 
            len(self.animals) == 3):
            for name, data in self.animals.items():
                if not data[1]:
                    self.state = State.SEARCHING
                    return
            self.state = State.COMPLETE
            return
        self.state = State(self.state.value + 1)
        time.sleep(3.0)
    
    def rotate(self):
        rq = self.robot_pose.orientation
        euler = list(tf_transformations.euler_from_quaternion([
            rq.x, rq.y, rq.z, rq.w
        ]))
        if self.state == State.SEARCH_TRANSLATING_DONE:
            euler[2] += pi / 2
        else:
            euler[2] -= pi
        self.state = State(self.state.value + 1)
        quat = tf_transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        path = Path()
        ps = PoseStamped()
        ps.pose.position = self.robot_pose.position
        ps.pose.orientation.x = quat[0] 
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        path.poses.append(ps)
        self.path_pub.publish(path)

    def distance(self, a, b):
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def nav_animal(self, name):
        self.state = State.ANIMAL_NAVIGATING
        self.animals[name][1] = True
        point = self.animals[name][0]
        path = self.find_path_around_animal(Point(x=point[0], y=point[1]))
        self.path_pub.publish(path)

    def find_path_around_animal(self, point:Point):

        loc = PoseStamped(pose=self.robot_pose)

        path = Path()

        rq = self.robot_pose.orientation
        r_euler = tf_transformations.euler_from_quaternion([rq.x, rq.y, rq.z, rq.w])
        rrq = tf_transformations.quaternion_from_euler(r_euler[0], r_euler[1], r_euler[2] + pi)
        rq_reverse = Quaternion()
        rq_reverse.x = rrq[0]
        rq_reverse.y = rrq[1]
        rq_reverse.z = rrq[2]
        rq_reverse.w = rrq[3]

        buoy_pos = point

        points = [
            (buoy_pos.x + 5, buoy_pos.y),
            (buoy_pos.x - 5, buoy_pos.y),
            (buoy_pos.x, buoy_pos.y + 5),
            (buoy_pos.x, buoy_pos.y - 5)
        ]

        points = list(
            LoopPoint('x' if i < 2 else 'y', p, MultiGates.distance(MultiGates.xy_to_pose_stamped(p, rq), loc))
            for i, p in enumerate(points)
        )

        closest_i = MultiGates.find_closest_index(points)
        path.poses.append(MultiGates.xy_to_pose_stamped(points[closest_i].xy, rq))
        points[closest_i].used = True

        for i, point in enumerate(points):
            if point.used:
                continue
            if point.change is not points[closest_i].change:
                closest_i = i
                path.poses.append(MultiGates.xy_to_pose_stamped(point.xy, rq))
                point.used = True
                break

        for i, point in enumerate(points):
            if point.used:
                continue
            if point.change is not points[closest_i].change:
                closest_i = i
                path.poses.append(MultiGates.xy_to_pose_stamped(point.xy, rq_reverse))
                point.used = True
                break
        
        for point in points:
            if point.used:
                continue
            path.poses.append(MultiGates.xy_to_pose_stamped(point.xy, rq_reverse))
            break

        # final = MultiGates.xy_to_pose_stamped((path.poses[0].pose.position.x, path.poses[0].pose.position.y), rq_reverse)
        # path.poses.append(final)

        # We can later remove this and have the robot choose a random gate to go through
        path.poses.append(loc)

        return path

def main(args=None):
    rclpy.init(args=args)
    node = WildlifeEncounterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()