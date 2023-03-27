import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Pose, PointStamped
from virtuoso_msgs.srv import Channel, Rotate, LidarBuoy
from .finals_states import State
from ...utils.channel_nav.channel_nav import ChannelNavigation
from ...utils.geometry_conversions import point_to_pose_stamped
from virtuoso_perception.utils.geometry_msgs import do_transform_point
import time
import tf_transformations
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time

class FinalsNode(Node):

    def __init__(self):
        super().__init__('autonomy_finals')

        self.declare_parameters(namespace='', parameters=[
            ('lidar_frame', ''),
            ('t1_extra_forward_nav', 0.0),
            ('t1_final_extra_forward_nav', 0.0),
            ('t1_gate_buoy_max_dist', 0.0),
            ('t2_enter_distance', 0.0),
            ('t2_backing_distance', 0.0),
            ('t3_direction', ''),
            ('t3_explore_orientation', []),
            ('t3_loop_explore_initial_nav_distance', 0.0),
            ('t3_loop_explore_find_attempts', 0),
            ('t3_loop_explore_extra_nav_distance', 0.0),
            ('t3_loop_explore_buoy_max_dist', 0.0),
            ('t3_gate_explore_initial_nav_distance', 0.0),
            ('t3_gate_buoy_max_dist', 0.0),
            ('t3_gate_explore_find_attempts', 0),
            ('t3_gate_explore_extra_nav_distance', 0.0)
        ])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.t3_explore_orientation = self.get_parameter('t3_explore_orientation').value

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
        self.trans_pub = self.create_publisher(Point, '/navigation/translate', 10)
        self.pose_pub = self.create_publisher(Pose, '/navigation/set_single_waypoint', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)

        self.rotate_client = self.create_client(Rotate, 'rotate')
        self.rotate_call = None
        
        self.state = State.START

        self.channel_client = self.create_client(Channel, 'channel')
        self.channel_call = None

        self.lidar_buoy_client = self.create_client(LidarBuoy, 'perception/lidar_buoy')
        self.lidar_buoy_call = None

        self.robot_pose:PoseStamped = None

        self.t1_channels_completed = 0
        self.t1_final_pose:PoseStamped = None

        self.find_attempts = 0
        self.loop_buoy_loc:Point = None

        self.create_timer(1.0, self.execute)

    def odom_callback(self, msg:Odometry):
        self.robot_pose = PoseStamped(pose=msg.pose.pose)
    
    def nav_success_callback(self, msg:PoseStamped):
        if self.state == State.T1_NAVIGATING:
            if self.t1_channels_completed < 2:
                time.sleep(2.0)
                self.t1_nav_forward()
            else:
                if self.t3_explore_orientation[0] == 0:
                    self.t3_save_explore_orientation(msg) 
                time.sleep(2.0)
                self.t1_final_nav_forward()
        elif self.state == State.T1_EXTRA_FORWARD_NAV:
            time.sleep(5.0)
            self.state = State.T1_FINDING_NEXT_GATE
        elif self.state == State.T1_FINAL_EXTRA_FORWARD_NAV:
            self.t1_final_pose = msg
            self.t2_enter()
        elif self.state == State.T2_ENTERING:
            time.sleep(2.0) 
            self.t2_exit()
        elif self.state == State.T2_BACKING:
            time.sleep(5.0)
            self.t3_initial_nav()
        elif self.state == State.T3_LOOP_EXPLORE_INITIAL_NAVIGATION:
            self.state = State.T3_LOOP_EXPLORE_ROTATING
        elif self.state == State.T3_LOOP_EXPLORE_EXTRA_NAVIGATION:
            time.sleep(5.0)
            self.t3_find_loop()
        elif self.state == State.T3_LOOP_APPROACH:
            self.t3_loop_buoy_rotate()
        elif self.state == State.T3_GATE_EXPLORE_NAVIGATION:
            self.t3_find_gate()
        elif self.state == State.T3_GATE_EXPLORE_EXTRA_NAVIGATION:
            self.t3_find_gate()
    
    def t3_find_gate(self):
        if self.channel_call is not None:
            return
        
        req = Channel.Request()
        req.left_color = 'red'
        req.right_color = 'green'
        req.use_lidar = True
        req.use_camera = False
        req.max_dist_from_usv = self.get_parameter('t3_gate_buoy_max_dist').value

        self.channel_call = self.channel_client.call_async(req)
        self.channel_call.add_done_callback(self.t3_channel_response)
    
    def t3_channel_response(self, future):
        result:Channel.Response = future.result()
        self.get_logger().info(f'response: {result}')

        null_point = Point(x=0.0,y=0.0,z=0.0)

        self.channel_call = None
        
        if result.left == null_point or result.right == null_point:
            self.get_logger().info('No Gate Found')
            self.find_attempts += 1
            if self.find_attempts >= self.get_parameter('t3_gate_explore_find_attempts').value:
                self.t3_channel_exploration_nav_forward()
            else:
                self.t3_find_gate()
        
        channel = (
            point_to_pose_stamped(result.left),
            point_to_pose_stamped(result.right)
        )

        mid = ChannelNavigation.find_midpoint(channel[0], channel[1], self.robot_pose)

        path = Path()
        path.poses.append(mid)

        self.state = State.T3_GATE_ENTER
        self.path_pub.publish(path)

    def t3_channel_exploration_nav_forward(self):
        self.find_attempts = 0
        self.state = State.T3_GATE_EXPLORE_EXTRA_NAVIGATION
        self.trans_pub.publish(Point(x=self.get_parameter('t3_gate_explore_extra_nav_distance').value))
        
    def t3_loop_buoy_rotate(self):
        if self.rotate_call is not None:
            return
        
        self.state = State.T3_LOOP_ORIENTING

        req = Rotate.Request()
        req.goal.z = -1 * math.pi / 2

        self.rotate_call = self.rotate_client.call_async(req)
        self.rotate_call.add_done_callback(self.rotate_callback)
    
    def find_lidar_to_map_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', self.get_parameter('lidar_frame').value, Time()
            )
            self.get_logger().info('got lidar to map transform')
            return trans
        except Exception:
            return None
    
    def t3_find_loop(self):
        
        if self.lidar_buoy_call is not None:
            return
        
        msg = LidarBuoy.Request()

        self.lidar_buoy_call = self.lidar_buoy_client.call_async(msg)
        self.lidar_buoy_call.add_done_callback(self.lidar_buoy_callback)
    
    def lidar_buoy_callback(self, future):
        result = future.result()
        # self.get_logger().info(f'response: {result}')

        self.lidar_buoy_call = None

        self.find_attempts += 1

        buoys = result.buoys

        if len(buoys.boxes) == 0:
            self.get_logger().info('No lidar buoys found')
            if self.find_attempts >= self.get_parameter('t3_loop_explore_find_attempts').value:  
                self.t3_loop_explore_nav_forward()
            return

        closest = None 
        closest_dist = None
        for buoy in buoys.boxes:
            dist = math.sqrt(buoy.centroid.x**2 + buoy.centroid.y**2)
            if (closest is None or dist < closest_dist) and dist < self.get_parameter('t3_loop_explore_buoy_max_dist').value:
                closest = buoy
                closest_dist = dist
        
        if closest is None:
            self.get_logger().info('No buoys within distance')
            return
        
        point = Point(x=closest.centroid.x,y=closest.centroid.y)
        ps = PointStamped(point=point)

        lidar_to_map = self.find_lidar_to_map_transform()

        if lidar_to_map is None:
            self.get_logger().info('No lidar to map')
            return
        
        map_point = do_transform_point(ps, lidar_to_map)

        self.loop_buoy_loc = map_point.point
        
        self.state = State.T3_LOOP_APPROACH
        self.find_attempts = 0

        pose = Pose()
        pose.position.x = point.x - 1
        self.pose_pub.publish(pose)
    
    def t3_loop_explore_nav_forward(self):
        self.find_attempts = 0
        self.state = State.T3_LOOP_EXPLORE_EXTRA_NAVIGATION
        pose = Pose()
        pose.position.y = -1 * self.get_parameter('t3_loop_explore_extra_nav_distance').value
        self.pose_pub.publish(pose)
    
    def t3_initial_nav(self):
        self.state = State.T3_LOOP_EXPLORE_INITIAL_NAVIGATION
        pose = Pose()
        pose.position.x = self.get_parameter('t3_loop_explore_initial_nav_distance').value
        self.pose_pub.publish(pose)
    
    def find_perp_orientation(self, orientation):
        euler = list(tf_transformations.euler_from_quaternion(orientation))
        if self.get_parameter('t3_direction').value == 'right':
            euler[2] -= (math.pi / 2)
        else:
            euler[2] += (math.pi / 2)
        
        if euler[2] > math.pi:
            euler[2] -= 2*math.pi
        elif euler[2] < -math.pi:
            euler[2] += 2*math.pi
        
        quat = tf_transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

        q = Quaternion()
        q.x = quat[0]
        q.y = quat[1]
        q.z = quat[2]
        q.w = quat[3]

        return q
    
    def t3_save_explore_orientation(self, pose:PoseStamped):
        self.get_logger().info('using new orientation')
        self.t3_explore_orientation = list()
        self.t3_explore_orientation.append(pose.pose.orientation.x)
        self.t3_explore_orientation.append(pose.pose.orientation.y)
        self.t3_explore_orientation.append(pose.pose.orientation.z)
        self.t3_explore_orientation.append(pose.pose.orientation.w)
    
    def t2_exit(self):
        self.state = State.T2_BACKING
        orientation = self.find_perp_orientation(self.t3_explore_orientation)
        self.t1_final_pose.pose.orientation = orientation
        path = Path()
        path.poses.append(self.t1_final_pose)
        self.path_pub.publish(path)
        
    def t2_enter(self):
        self.state = State.T2_ENTERING
        self.trans_pub.publish(Point(x=self.get_parameter('t2_enter_distance').value))
    
    def t1_final_nav_forward(self):
        self.state = State.T1_FINAL_EXTRA_FORWARD_NAV
        self.trans_pub.publish(Point(x=self.get_parameter('t1_final_extra_forward_nav').value))
    
    def t1_nav_forward(self):
        self.state = State.T1_EXTRA_FORWARD_NAV
        self.trans_pub.publish(Point(x=self.get_parameter('t1_extra_forward_nav').value))
    
    def execute(self):
        self.get_logger().info(str(self.state))

        if self.state == State.START:
            self.state = State.T1_FINDING_NEXT_GATE
        elif self.state == State.T1_FINDING_NEXT_GATE:
            self.t1_nav_to_channel_midpoint()
        elif self.state == State.T3_LOOP_EXPLORE_ROTATING:
            self.t3_rotate()
        elif self.state == State.T3_LOOP_EXPLORE_FINDING:
            self.t3_find_loop()
    
    def t3_rotate(self):
        if self.rotate_call is not None:
            return

        req = Rotate.Request()
        req.goal.z = math.pi / 2

        self.rotate_call = self.rotate_client.call_async(req)
        self.rotate_call.add_done_callback(self.rotate_callback)
    
    def rotate_callback(self, future):
        result = future.result()
        self.get_logger().info(f'response: {result}')

        self.rotate_call = None

        if self.state == State.T3_LOOP_EXPLORE_ROTATING:
            self.state = State.T3_LOOP_EXPLORE_FINDING
        elif self.state == State.T3_LOOP_ORIENTING:
            self.t3_initial_forward_nav()
    
    def t3_initial_forward_nav(self):
        self.state = State.T3_GATE_EXPLORE_NAVIGATION
        pose = Pose()
        pose.position.x = self.get_parameter('t3_gate_explore_initial_nav_distance').value
        self.pose_pub.publish(pose)
    
    def t1_nav_to_channel_midpoint(self):

        if self.robot_pose is None:
            return
        if self.channel_call is not None:
            return
        
        req = Channel.Request()
        req.left_color = 'red'
        req.right_color = 'green'
        req.use_lidar = True
        req.use_camera = False
        req.max_dist_from_usv = self.get_parameter('t1_gate_buoy_max_dist').value

        self.channel_call = self.channel_client.call_async(req)
        self.channel_call.add_done_callback(self.t1_channel_response)
    
    def t1_channel_response(self, future):
        result:Channel.Response = future.result()
        self.get_logger().info(f'response: {result}')

        null_point = Point(x=0.0,y=0.0,z=0.0)

        self.channel_call = None
        
        if result.left == null_point or result.right == null_point:
            self.get_logger().info('No Gate Found')
            return
        
        channel = (
            point_to_pose_stamped(result.left),
            point_to_pose_stamped(result.right)
        )

        mid = ChannelNavigation.find_midpoint(channel[0], channel[1], self.robot_pose)

        path = Path()
        path.poses.append(mid)

        self.state = State.T1_NAVIGATING
        self.t1_channels_completed += 1
        self.path_pub.publish(path)
    

def main(args=None):
    rclpy.init(args=args)

    node = FinalsNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()