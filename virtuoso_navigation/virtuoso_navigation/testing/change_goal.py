import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
import random

# Node that publishes a new goal every 10 seconds
# Should notice that whenever a new goal is published, a new plan is generated

class TestChangeGoal(Node):

    def __init__(self):
        super().__init__('test_change_goal')
        self.pub = self.create_publisher(PoseStamped, '/virtuoso_navigation/set_goal', 10)

        self.timer = self.create_timer(10, self.publish_new_point)

    
    def publish_new_point(self):
        
        pose_stamped = PoseStamped()
        
        pose = Pose()
        pose.position.x = float(random.randint(-10, 10) )
        pose.position.y = float(random.randint(-10, 10)) 
        pose.position.z = 0.0
        pose.orientation = Quaternion()

        pose_stamped.pose = pose

        self.pub.publish(pose_stamped)



def main(args=None):
    
    rclpy.init(args=args)

    node = TestChangeGoal()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()