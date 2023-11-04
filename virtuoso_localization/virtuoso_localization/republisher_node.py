import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

from tf2_ros.buffer import Buffer

from tf2_ros.transform_listener import TransformListener 
from tf2_ros import TransformBroadcaster

class RepublisherNode(Node):

    def __init__(self):
        super().__init__('localization_republisher')
        self.measured_IMU = Imu()
        self.gps_fix = NavSatFix()
        self.gps_fix_vel = Vector3Stamped()
        self.stateEstimate = Odometry()
        self.IMU_ready = False
        self.GPS_ready = False
        self.GPS_vel_ready = False

        self.declare_parameters(namespace='', parameters=[
            ('imu_topic', ''),
            ('gps_topic', ''),
            ('gps_vel_topic', '')
        ])
        

        self.imuPublisher = self.create_publisher(Imu, '/navsat/imu', 10)
        self.gpsPublisher = self.create_publisher(NavSatFix, '/navsat/gps', 10)
        
        #subscribe to wamv sensor data
        self.imu_subscriber = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.imu_callback,
            10)
        self.gps_fix_subscriber = self.create_subscription(
            NavSatFix,
            self.get_parameter('gps_topic').value,
            self.gps_fix_callback,
            10)
        self.gps_fix_vel_subscriber = self.create_subscription(
            Vector3Stamped,
            self.get_parameter('gps_vel_topic').value,
            self.gps_fix_vel_callback,
            10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)

    def imu_callback(self, msg:Imu):
        self.measured_IMU = msg
        self.measured_IMU.header.frame_id = 'wamv/wamv/imu_wamv_link'
        self.IMU_ready = True
        self.state_estimation()

        
    def gps_fix_callback(self, msg:NavSatFix):
        self.gps_fix = msg
        self.gps_fix.header.frame_id = 'wamv/wamv/gps_wamv_link'
        self.GPS_ready = True
        self.state_estimation()
        
    def gps_fix_vel_callback(self, msg):
        self.gps_fix_vel = msg
        self.GPS_vel_ready = True
        self.state_estimation()
    
    #if all the data is ready, publish it to the ekf and navsattransform nodes
    def state_estimation(self):
        if not self.GPS_ready or not self.IMU_ready:
            return

        self.imuPublisher.publish(self.measured_IMU)
        self.gpsPublisher.publish(self.gps_fix)
        
def main(args=None):
    rclpy.init(args=args)

    continual_EKF = RepublisherNode()

    rclpy.spin(continual_EKF)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    continual_EKF.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
