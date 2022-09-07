import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from ublox_ubx_msgs.msg import UBXNavCov
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class gx3Republish(Node):

    def __init__(self):
        super().__init__('gx3_republish')
        
        self.imu_data = Imu()

        
        self.imuPublisher = self.create_publisher(Imu, '/wamv/sensors/imu/imu/data', 10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        #subscribe to wamv sensor data
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_profile=qos_profile)    
        self.imu_callback

        #self.get_logger().info('intializing ') 
        
    def imu_callback(self, msg):
        #msg2 = Imu()
        #self.get_logger().info('imu msg received ') 
        self.imu_data = msg
        self.imuPublisher.publish(self.imu_data)


        
def main(args=None):
    rclpy.init(args=args)

    gx3_republish = gx3Republish()

    rclpy.spin(gx3_republish)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gx3_republish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
