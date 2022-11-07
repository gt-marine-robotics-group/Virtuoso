import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from .wildlife_encounter_states import State
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from math import sqrt

class WildlifeEncounterNode(Node):

    possible_animals = ['Platypus', 'Crocodile', 'Turtle']

    def __init__(self):
        super().__init__('autonomy_wildlife_encounter')

        self.path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/virtuoso_navigation/success',
            self.nav_success_callback, 10)
        self.buoys_sub = self.create_subscription(BoundingBoxArray, '/buoys/bounding_boxes', 
            self.buoys_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)
        
        self.state = State.SEARCHING

        self.robot_pose = None
        self.animals = dict()

        
    
    def odom_callback(self, msg:Odometry):
        self.robot_pose = msg.pose.pose
    
    def buoys_callback(self, msg:BoundingBoxArray):
        for buoy in msg.boxes:
            if len(self.animals) > 2: return
            for animal, loc in self.animals:
                if self.distance(loc, (buoy.centroid.x, buoy.centroid.y)) > 3:
                    break
            else:
                index = WildlifeEncounterNode.possible_animals[len(self.animals) - 1]
                self.animals[index] = (buoy.centroid.x, buoy.centroid.y)
    
    def nav_success_callback(self, msg):
        if self.state == State.ANIMAL_NAVIGATING:
            if len(self.animals) == 3:
                self.state = State.COMPLETE
            else:
                self.state = State.SEARCHING
        else:
            self.state = State.SEARCHING
    
    def distance(self, a, b):
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def main(args=None):
    rclpy.init(args=args)
    node = WildlifeEncounterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()