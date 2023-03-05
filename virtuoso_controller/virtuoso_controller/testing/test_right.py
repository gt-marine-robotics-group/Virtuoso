import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

class TestRight(Node):

    def __init__(self):
        super().__init__('test_controller_right')

        self.navigateToPoint = Bool()
        self.navigateToPoint.data = True
        self.basicTargetForceX = Float32()
        self.basicTargetForceX.data = 0.0
        self.basicTargetForceY = Float32()
        self.basicTargetForceY.data = -0.5
        self.basicTorque = Float32()
        self.basicTorque.data = 0.0
        self.velForceX = Float32()
        self.velForceX.data = 0.0
        self.velForceY = Float32()
        self.velForceY.data = -0.5
        self.velTorque = Float32()
        self.velTorque.data = 0.0

        self.navigateToPoint_pub = self.create_publisher(Bool, '/navigation/navigateToPoint', 10)
        self.basicTargetForceX_pub = self.create_publisher(Float32, 'controller/basic_pid/targetForceX', 10)
        self.basicTargetForceY_pub = self.create_publisher(Float32, 'controller/basic_pid/targetForceY', 10)
        self.basicTorque_pub = self.create_publisher(Float32, 'controller/basic_pid/targetTorque', 10)
        self.velForceX_pub = self.create_publisher(Float32, 'controller/velocity_pid/targetForceX', 10)
        self.velForceY_pub = self.create_publisher(Float32, 'controller/velocity_pid/targetForceY', 10)
        self.velTorque_pub = self.create_publisher(Float32, 'controller/velocity_pid/targetTorque', 10)

        self.timer = self.create_timer(0.5, self.send_command)
    
    def send_command(self):
        self.get_logger().info('SENDING COMMAND')
        self.navigateToPoint_pub.publish(self.navigateToPoint)
        self.basicTargetForceX_pub.publish(self.basicTargetForceX)
        self.basicTargetForceY_pub.publish(self.basicTargetForceY)
        self.basicTorque_pub.publish(self.basicTorque)
        self.velForceX_pub.publish(self.velForceX)
        self.velForceY_pub.publish(self.velForceY)
        self.velTorque_pub.publish(self.velTorque)
    
def main(args=None):
    rclpy.init(args=args)

    test = TestRight()

    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
