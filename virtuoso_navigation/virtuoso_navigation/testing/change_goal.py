import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Quaternion

class TestChangeGoal(Node):

    def __init__(self):
        super().__init__('test_change_goal')
        self.pub = self.create_publisher(PoseStamped, '/virtuoso_navigation/set_goal', 10)

        pose_stamped = PoseStamped()
        
        pose = Pose()
        pose.position.x = -5.0
        pose.position.y = 5.0
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