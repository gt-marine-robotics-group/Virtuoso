import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from robot_localization.srv import FromLL
from ros_gz_interfaces.msg import ParamVec
from geographic_msgs.msg import GeoPoint
from virtuoso_autonomy.utils.task_info import get_state
import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class WayfindingNode(Node):

    def __init__(self):
        super().__init__('wayfinding')

        self.transformed_waypoints = Path()
        self.raw_waypoints = None
        self.executing = False

        self.transform_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.task_info_sub = self.create_subscription(ParamVec, '/vrx/task/info', 
            self.task_info_callback, 10)
        
        self.waypoints_sub = self.create_subscription(PoseArray, '/vrx/wayfinding/waypoints',
            self.waypoints_callback, 10)

        self.fromLL_cli = self.create_client(FromLL, '/fromLL')

        self.path_pub = self.create_publisher(Path, '/navigation/set_waypoints', 10)

        self.create_timer(1.0, self.execute)

    def task_info_callback(self, msg:ParamVec):
        self.state = get_state(msg)
    
    def waypoints_callback(self, msg:PoseArray):
        self.raw_waypoints = msg
    
    def execute(self):

        if self.executing:
            return
        
        if self.raw_waypoints is None:
            return
        
        if len(self.transformed_waypoints.poses) == 0:
            self.executing = True
            self.transform_waypoints()
            return
        
        if len(self.transformed_waypoints.poses) < len(self.raw_waypoints.poses):
            return
        
        self.executing = True
        self.get_logger().info(str(self.transformed_waypoints))
        self.path_pub.publish(self.transformed_waypoints)
    
    def transform_waypoints(self):

        from_frame_rel = 'utm'
        to_frame_rel = 'odom'
        
        try:
             now = rclpy.time.Time()
             trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
        except TransformException as ex:
             self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
             self.executing = False
             return
        self.quatUTMtoODOM = trans.transform.rotation

        def ll_callback(future):
            self.get_logger().info(f'finished transform {self.transform_count}')
            point = future.result().map_point
            pose_stamped = PoseStamped()
            pose = Pose()
            pose.position = point

            o = self.raw_waypoints.poses[self.transform_count].orientation
            q_goal = [o.x, o.y, o.z, o.w]
            
            q_utm_to_odom = [self.quatUTMtoODOM.x, self.quatUTMtoODOM.y, self.quatUTMtoODOM.z, self.quatUTMtoODOM.w]
            q_final = tf_transformations.quaternion_multiply(q_utm_to_odom, q_goal)
            pose.orientation.x = q_final[0]
            pose.orientation.y = q_final[1]
            pose.orientation.z = q_final[2]
            pose.orientation.w = q_final[3]
            # pose.orientation = self.raw_waypoints.poses[self.transform_count].orientation
            pose_stamped.pose = pose

            self.transformed_waypoints.poses.append(pose_stamped)
            
            self.transform_count += 1
            if (self.transform_count >= len(self.raw_waypoints.poses)):
                self.executing = False
                return
            ll = self.create_ll_future()
            ll.add_done_callback(ll_callback)

        ll = self.create_ll_future()
        ll.add_done_callback(ll_callback)
    
    def create_ll_future(self):
        self.req = FromLL.Request()
        self.req.ll_point = GeoPoint()
        p = self.raw_waypoints.poses[self.transform_count].position
        self.req.ll_point.longitude = p.y
        self.req.ll_point.latitude = p.x
        self.req.ll_point.altitude = p.z
        return self.fromLL_cli.call_async(self.req)
    

def main(args=None):
    
    rclpy.init(args=args)

    node = WayfindingNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()