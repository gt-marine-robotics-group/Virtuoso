import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer

from tf2_ros.transform_listener import TransformListener 
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped

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
        

        self.imuPublisher = self.create_publisher(Imu, '/navsat/imu', 10)
        self.gpsPublisher = self.create_publisher(NavSatFix, '/navsat/gps', 10)
        
        #subscribe to wamv sensor data
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        self.declare_parameter('sim_time', '')

        # self.create_timer(.5, self.state_estimation)

    def imu_callback(self, msg):
        #msg2 = Imu()
        self.measured_IMU = msg
        self.IMU_ready = True
        self.state_estimation()

        
    def gps_fix_callback(self, msg):
        #msg2 = Imu()
        self.gps_fix = msg
        self.GPS_ready = True
        self.state_estimation()
        
    def gps_fix_vel_callback(self, msg):
        self.gps_fix_vel = msg
        self.GPS_vel_ready = True
        self.state_estimation()
        
    def ekf_callback(self, msg):
         self.get_logger().info(str(msg))
    
    #if all the data is ready, publish it to the ekf and navsattransform nodes
    def state_estimation(self):
        # self.get_logger().info(str(self.get_parameter('sim_time').value))
        if not self.GPS_ready or not self.IMU_ready:
            return

        self.imuPublisher.publish(self.measured_IMU)
        self.gpsPublisher.publish(self.gps_fix)

        self.IMU_ready = False
        self.GPS_ready = False
        self.GPS_vel_ready = False        

        if self.get_parameter('sim_time').value:
            return

        # self.get_logger().info('SIM TIME IS FALSE')

        trans = None
    
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'wamv/base_link',
                'utm',
                now)
            #self.get_logger().info('transform success')
        except TransformException as ex:
            #self.get_logger().info(
            #    f'Could not transform utm to base link: {ex}')
            return

        trans2 = None

        try:
            now = rclpy.time.Time()
            trans2 = self.tf_buffer.lookup_transform(
                'wamv/base_link',
                'ubx',
                now)
            
        except TransformException as ex:
            #self.get_logger().info(
            #    f'Could not transform ubx to base link: {ex}')
            return
            

        static_transformStamped = TransformStamped()
        
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'ubx_utm'
        static_transformStamped.child_frame_id = 'wamv/base_link'
        
        static_transformStamped.transform.translation.x = trans2.transform.translation.x
        static_transformStamped.transform.translation.y = trans2.transform.translation.y
        static_transformStamped.transform.translation.z = trans2.transform.translation.z
        
        static_transformStamped.transform.rotation.x = trans.transform.rotation.x
        static_transformStamped.transform.rotation.y = trans.transform.rotation.y
        static_transformStamped.transform.rotation.z = trans.transform.rotation.z
        static_transformStamped.transform.rotation.w = trans.transform.rotation.w

        
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
