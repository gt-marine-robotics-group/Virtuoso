import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from ublox_ubx_msgs.msg import UBXNavHPPosLLH, UBXNavVelNED
from ublox_ubx_msgs.msg import UBXNavCov
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from tf2_ros import TransformBroadcaster


from geometry_msgs.msg import TransformStamped

class f9pGPSRepublish(Node):

    def __init__(self):
        super().__init__('f9p_gps_republish')
        
        self.gps_fix = UBXNavHPPosLLH()
        self.gps_cov = UBXNavCov()
        self.gps_vel = UBXNavVelNED()
        
        self.GPS_ready = False
        self.GPS_Cov_ready = False
        self.GPS_vel_ready = False

        
        self.gpsPublisher = self.create_publisher(NavSatFix, '/wamv/sensors/gps/gps/fix', 10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.gpsVelPublisher = self.create_publisher(TwistWithCovarianceStamped, '/wamv/sensors/gps/gps/vel', 10)
        
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
        self.gps_vel_subscriber = self.create_subscription(
            UBXNavVelNED,
            '/ubx_nav_vel_ned',
            self.gps_vel_callback,
            qos_profile=qos_profile)   
        self.gps_fix_subscriber
        self.gps_cov_subscriber
        self.gps_vel_subscriber
        self.get_logger().info('intializing ') 
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)     
           
    def gps_fix_callback(self, msg):
        self.gps_fix = msg
        if((not msg.invalid_lat) and (not msg.invalid_lon)):
             self.GPS_ready = True
        self.publish_gps()
             
    def gps_cov_callback(self, msg):
        self.gps_cov = msg
        self.GPS_Cov_ready = True
        self.publish_gps()
        
    def gps_vel_callback(self, msg):
        self.gps_vel = msg
        self.GPS_vel_ready = True
        self.publish_gps()
                 
    #if all the data is ready, publish it to the ekf and navsattransform nodes
    def publish_gps(self):
        #if(self.GPS_ready and self.GPS_Cov_ready and self.GPS_vel_ready):
             navsatmsg = NavSatFix()
             
             navsatmsg.header = self.gps_fix.header
             
             navsatmsg.status.status = 1
             
             navsatmsg.latitude = float(self.gps_fix.lat*10**(-7) + self.gps_fix.lat_hp*10**(-9))
             navsatmsg.longitude = float(float(self.gps_fix.lon)*10**(-7) + float(self.gps_fix.lon_hp)*10**(-9))
             navsatmsg.altitude = float(self.gps_fix.height + self.gps_fix.height_hp*0.1)*10**(-3)
             
             #navsatmsg.position_covariance = [self.gps_cov.pos_cov_ee, self.gps_cov.pos_cov_ne, -self.gps_cov.pos_cov_ed, self.gps_cov.pos_cov_ne, self.gps_cov.pos_cov_nn, -self.gps_cov.pos_cov_nd, -self.gps_cov.pos_cov_ed, -self.gps_cov.pos_cov_nd, self.gps_cov.pos_cov_dd]
             
             navsatmsg.position_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
             
             navsatmsg.position_covariance_type = 3
             self.gpsPublisher.publish(navsatmsg)
             self.GPS_ready = False
             self.GPS_Cov_ready = False
             self.GPS_vel_ready = False
             
             twistmsg = TwistWithCovarianceStamped()
             
             twistmsg.header = self.gps_vel.header
             twistmsg.header.frame_id = 'ubx_utm'
             
             twistmsg.twist.twist.linear.x = self.gps_vel.vel_e/100.0
             twistmsg.twist.twist.linear.y = self.gps_vel.vel_n/100.0
             twistmsg.twist.twist.linear.z = -self.gps_vel.vel_d/100.0        
             
             twistmsg.twist.covariance[0] = 0.01
             twistmsg.twist.covariance[7] = 0.01
             twistmsg.twist.covariance[14] = 0.01
             
             self.gpsVelPublisher.publish(twistmsg)
             
             trans = None
     
             try:
                  now = rclpy.time.Time()
                  trans = self.tf_buffer.lookup_transform(
                       'utm',
                       'wamv/base_link',
                       now)
             except TransformException as ex:
                  self.get_logger().info(
                       f'Could not transform base link to utm: {ex}')
                  return
                  
             trans2 = None
     
             try:
                  now = rclpy.time.Time()
                  trans2 = self.tf_buffer.lookup_transform(
                       'ubx', 
                       'wamv/base_link',
                       now)
                  
             except TransformException as ex:
                  self.get_logger().info(
                       f'Could not transform base link to utm: {ex}')
                  return
                  

             static_transformStamped = TransformStamped()
             
             static_transformStamped.header.stamp = self.get_clock().now().to_msg()
             static_transformStamped.header.frame_id = 'wamv/base_link'
             static_transformStamped.child_frame_id = 'ubx_utm'
             
             static_transformStamped.transform.translation.x = trans2.transform.translation.x
             static_transformStamped.transform.translation.y = trans2.transform.translation.y
             static_transformStamped.transform.translation.z = trans2.transform.translation.z
             
             static_transformStamped.transform.rotation.x = -trans.transform.rotation.x
             static_transformStamped.transform.rotation.y = -trans.transform.rotation.y
             static_transformStamped.transform.rotation.z = -trans.transform.rotation.z
             static_transformStamped.transform.rotation.w = trans.transform.rotation.w
             
             self.broadcaster.sendTransform(static_transformStamped)
        
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
