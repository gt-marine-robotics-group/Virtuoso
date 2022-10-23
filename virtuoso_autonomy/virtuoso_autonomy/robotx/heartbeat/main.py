import rclpy
from rclpy.node import Node
from virtuoso_msgs.msg import Task
from geometry_msgs.msg import PoseStamped, Pose
from geographic_msgs.msg import GeoPoseStamped
from robot_localization.srv import FromLL
from std_msgs.msg import String
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
import datetime

class Heartbeat(Node):

    def __init__(self):
        super().__init__('heartbeat')
        
        #self.gps_ready = True
        #self.lat = 99.111111111
        #self.lon = 99.111111111
        
        self.gps_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_sub_callback, 10)
        self.system_mode_sub = self.create_subscription(Int8, '/wamv/nova/mode', self.system_mode_sub_callback, 10)
        self.gps_ready = False
        self.system_mode = 0


        self.heartbeat_pub = self.create_publisher(String, '/wamv/heartbeat', 10)

        self.heartbeat_timer = self.create_timer(1.0, self.send_heartbeat)

    
    def gps_sub_callback(self, msg):

        self.lat = msg.latitude
        self.lon = msg.longitude
        
        self.gps_ready = True
    
    def system_mode_sub_callback(self, msg):
        self.system_mode = msg.data      

    
    def send_heartbeat(self):
        if(self.gps_ready):
             msg = String()
             msg.data = "$RXHRB"
             teamid = "ROBOT"
             checksum = "11"
             crlf = "\r\n"
             lat_str = str(round(self.lat,5)) + ",N"
             lon_str = str(round(self.lon,5)) + ",E"
             system_mode = str(self.system_mode)
             uav_status = "1"
             time = self.get_clock().now().to_msg()
             dt = datetime.datetime.fromtimestamp(time.sec)
             aedt_date = str(dt.day) + str(dt.month) + str(dt.year)
             aedt_time = str(dt.hour) + str(dt.minute) + str(dt.second)
        
        
             msg.data = msg.data + "," + aedt_date + "," + aedt_time + "," + lat_str + "," + lon_str + "," + teamid + "," + system_mode + "," + uav_status + "*" + checksum +  crlf
             self.get_logger().info(msg.data)
             self.heartbeat_pub.publish(msg)


def main(args=None):
    
    rclpy.init(args=args)

    node = Heartbeat()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

