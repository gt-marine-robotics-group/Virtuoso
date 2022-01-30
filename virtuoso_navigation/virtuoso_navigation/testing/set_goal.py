import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class SetGoal(Node):

    def __init__(self):
        super().__init__('set_goal')
        
        self.pub = self.create_publisher(Point, '/path_finding/goal', 10)

        point = Point()
        point.x = 5.0
        point.y = 5.0

        self.pub.publish(point)


def main(args=None):
    
    rclpy.init(args=args)

    node = SetGoal()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()