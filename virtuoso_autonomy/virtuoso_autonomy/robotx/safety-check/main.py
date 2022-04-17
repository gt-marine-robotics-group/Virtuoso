import rclpy
from rclpy.node import Node

class SafetyCheck(Node):

    def __init__(self):
        super().__init__('safety-check')


def main(args=None):
    
    rclpy.init(args=args)

    node = SafetyCheck()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()