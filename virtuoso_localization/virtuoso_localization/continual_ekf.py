import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

#import pyproj
#import math


class continualEKF(Node):

    def __init__(self):
        super().__init__('continual_EKF')
        self.measured_IMU = Imu()
        self.gps_fix = NavSatFix()
        self.gps_fix_vel = Vector3Stamped()
        self.stateEstimate = Odometry()
        self.IMU_ready = False
        self.GPS_ready = False
        self.GPS_vel_ready = False
        
        #self.publisher_ = self.create_publisher(Odometry, '/navsat/odometry', 10)
        self.imuPublisher = self.create_publisher(Imu, '/navsat/imu', 10)
        self.gpsPublisher = self.create_publisher(NavSatFix, '/navsat/gps', 10)
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            10)
        self.gps_fix_subscriber = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_fix_callback,
            10)
        self.gps_fix_vel_subscriber = self.create_subscription(
            Vector3Stamped,
            '/wamv/sensors/gps/gps/fix_velocity',
            self.gps_fix_vel_callback,
            10)
        self.ekf_subscriber = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.ekf_callback,
            10)            
        self.imu_subscriber
        self.gps_fix_subscriber
        self.gps_fix_vel_subscriber

    def imu_callback(self, msg):
        #msg2 = Imu()
        self.measured_IMU = msg
        self.IMU_ready = True
        self.state_estimation()
        #x = self.measured_attitude.x
        #y = self.measured_attitude.y
        #z = self.measured_attitude.z
        #w = self.measured_attitude.w
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #self.publisher_.publish(msg2)
        #self.get_logger().info('Orientation:' + str(x) + ' ' + str(y) + ' ' + str(z) + ' ' + str(w))
        
    def gps_fix_callback(self, msg):
        #msg2 = Imu()
        self.gps_fix = msg
        self.GPS_ready = True
        self.state_estimation()
        #lat = self.gps_fix.latitude
        #longitude = self.gps_fix.longitude
        
        #self.get_logger().info('lat:' + str(lat) + ' long: ' + str(longitude))
        
    def gps_fix_vel_callback(self, msg):
        self.gps_fix_vel = msg
        self.GPS_vel_ready = True
        self.state_estimation()
        
    def ekf_callback(self, msg):
         self.get_logger().info(str(msg))
        
    def state_estimation(self):
        if(self.GPS_vel_ready, self.GPS_ready, self.IMU_ready):
             self.imuPublisher.publish(self.measured_IMU)
             self.gpsPublisher.publish(self.gps_fix)
             #self.publisher_.publish(self.stateEstimate)
             
             #self.get_logger().info(str(self.stateEstimate))
             self.IMU_ready = False
             self.GPS_ready = False
             self.GPS_vel_ready = False        

        
def main(args=None):
    rclpy.init(args=args)

    continual_EKF = continualEKF()

    rclpy.spin(continual_EKF)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    continual_EKF.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
