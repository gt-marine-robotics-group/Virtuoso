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


class f9pGPSRepublish(Node):

    def __init__(self):
        super().__init__('f9p_gps_republish')
        
        self.gps_fix = UBXNavHPPosLLH()
        self.gps_cov = UBXNavCov()
        
        self.GPS_ready = False
        self.GPS_Cov_ready = False

        
        self.gpsPublisher = self.create_publisher(NavSatFix, '/wamv/sensors/gps/gps/fix', 10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        #subscribe to wamv sensor data
        self.gps_fix_subscriber = self.create_subscription(
            UBXNavHPPosLLH,
            '/ubx_nav_hp_pos_llh',
            self.gps_fix_callback,
            qos_profile=qos_profile)
        self.gps_cov_subscriber = self.create_subscription(
            UBXNavCov,
            '/ubx_nav_cov',
            self.gps_cov_callback,
            qos_profile=qos_profile)       
        self.gps_fix_subscriber
        self.gps_cov_subscriber
        self.get_logger().info('intializing ') 
        
    def gps_fix_callback(self, msg):
        #msg2 = Imu()
        self.get_logger().info('gps msg received ') 
        self.gps_fix = msg
        if((not msg.invalid_lat) and (not msg.invalid_lon)):
             self.GPS_ready = True
        self.publish_gps()
             
    def gps_cov_callback(self, msg):
        #msg2 = Imu()
        self.get_logger().info('gps cov received ') 
        self.gps_cov = msg
        #if(not msg.pos_cor_valid):
        self.GPS_Cov_ready = True
        self.publish_gps()
                 
    #if all the data is ready, publish it to the ekf and navsattransform nodes
    def publish_gps(self):
        self.get_logger().info('checking ') 
        if(self.GPS_ready and self.GPS_Cov_ready):
             navsatmsg = NavSatFix();
             
             navsatmsg.header = self.gps_fix.header
             
             navsatmsg.status.status = 1
             
             navsatmsg.latitude = float(self.gps_fix.lon*10**(-7) + self.gps_fix.lon_hp*10**(-9))
             navsatmsg.longitude = float(float(self.gps_fix.lat)*10**(-7) + float(self.gps_fix.lat_hp)*10**(-9))
             navsatmsg.altitude = float(self.gps_fix.height + self.gps_fix.height_hp*0.1)*10**(-3)
             
             navsatmsg.position_covariance = [self.gps_cov.pos_cov_ee, self.gps_cov.pos_cov_ne, -self.gps_cov.pos_cov_ed, self.gps_cov.pos_cov_ne, self.gps_cov.pos_cov_nn, -self.gps_cov.pos_cov_nd, -self.gps_cov.pos_cov_ed, -self.gps_cov.pos_cov_nd, self.gps_cov.pos_cov_dd]
             
             navsatmsg.position_covariance_type = 3
             self.get_logger().info('publishing ') 
             self.gpsPublisher.publish(navsatmsg)
             self.GPS_ready = False
             self.GPS_Cov_ready = False


        
def main(args=None):
    rclpy.init(args=args)

    f9p_gps_republish = f9pGPSRepublish()

    rclpy.spin(f9p_gps_republish)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    f9p_gps_republish.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
