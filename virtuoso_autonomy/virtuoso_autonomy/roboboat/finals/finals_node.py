import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path, Odometry
from virtuoso_msgs.srv import Channel, DockCodesCameraPos, Rotate
from virtuoso_msgs.action import TaskWaypointNav, ApproachTarget, ShootBalls
from .finals_states import State
from ...utils.channel_nav.channel_nav import ChannelNavigation
from ...utils.geometry_conversions import point_to_pose_stamped
from ...utils.math import distance_pose_stamped
from ...utils.looping_buoy.looping_buoy import LoopingBuoy
import time
import tf_transformations
import math

class FinalsNode(Node):

    def __init__(self):
        super().__init__('autonomy_finals')

        self.declare_parameters(namespace='', parameters=[
            ('task_nums', []),
            ('docking_secs', 1),
            ('water_secs', 1),

            ('t1_auto', False),
            ('t2_auto', False),
            ('t3_auto', False),
            ('t4_auto', False),
            ('t5_auto', False),
            ('t6_auto', False),
            ('t7_auto', False),
            ('t8_auto', False),

            ('t1_extra_forward_nav', 0.0),
            ('t1_final_extra_forward_nav', 0.0),
            ('t1_gate_buoy_max_dist', 0.0),

            ('t2_trans_x', 0.0),
            ('t2_trans_y', 0.0),

            ('t3_target_color', ''),
            ('t3_camera', ''),
            ('t3_skip_line_up', False),
            ('t3_centering_movement_tranches', []),
            ('t3_centering_movement_values', []),
            ('t3_search_movement_value', 0.0),
            ('t3_approach_dist', 0.0),
            ('t3_approach_scan_width', 0.0),

            ('t4_gate_max_buoy_dist', 0.0),
            ('t4_loop_max_buoy_dist', 0.0),
            ('t4_gate_extra_forward_nav', 0.0),
            ('t4_final_extra_forward_nav', 0.0),
            ('t4_looping_radius', 0.0),

            ('t6_approach_dist', 0.0),
            ('t6_approach_scan_width', 0.0),
            ('t6_back_up_dist', 0.0),

            ('timeouts.t1_find_next_gate', 0),
            ('timeouts.t3_code_search', 0),
            ('timeouts.t3_approach', 0),
            ('timeouts.t4_gate', 0),
            ('timeouts.t4_loop', 0),
            ('timeouts.t6_approach', 0),
            ('timeouts.t6_rotate', 0),
            ('timeouts.t6_back_up', 0),

            ('timeout_responses.t1_find_next_gate_trans', 0.0),
            ('timeout_responses.t4_gate_trans', 0.0)
        ])

        self.t3_target_color = self.get_parameter('t3_target_color').value

        self.task_nums = self.get_parameter('task_nums').value

        self.curr_task = 0 # TEMP -1

        self.state = State.T1_FINDING_NEXT_GATE # TEMP State.START

        self.nav_client = ActionClient(self, TaskWaypointNav, 'task_waypoint_nav')
        self.nav_req = None
        self.nav_result = None

        self.path_pub = self.create_publisher(Path, '/navigation/set_waypoints', 10)
        self.trans_pub = self.create_publisher(Point, '/navigation/translate', 10)

        self.nav_success_sub = self.create_subscription(PoseStamped, '/navigation/success', 
            self.nav_success_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization/odometry', 
            self.odom_callback, 10)

        self.channel_client = self.create_client(Channel, 'channel')
        self.channel_call = None

        self.code_pos_client = self.create_client(DockCodesCameraPos, 
            f'{self.get_parameter("t3_camera").value}/find_dock_placard_offsets')
        self.code_pos_req = None

        self.approach_client = ActionClient(self, ApproachTarget, 'approach_target')
        self.approach_req = None
        self.approach_result = None

        self.rotate_client = self.create_client(Rotate, 'rotate')
        self.rotate_call = None

        self.ball_shooter_client = ActionClient(self, ShootBalls, 'shoot_balls')
        self.ball_shooter_req = None
        self.ball_shooter_result = None

        self.timeout_secs = 0
        self.docking_timeout_secs = 0

        self.channels_completed = 0

        self.t4_gate_midpoint = None

        self.t6_backup_time = 0
        self.t6_rotate_time = 0

        self.robot_pose:PoseStamped = None

        self.create_timer(1.0, self.execute)
    
    def odom_callback(self, msg:Odometry):
        self.robot_pose = PoseStamped(pose=msg.pose.pose)
    
    def nav_success_callback(self, msg:PoseStamped):
        self.timeout_secs = 0
        if self.state == State.T1_GATE_NAVIGATING:
            time.sleep(2.0)
            if self.channels_completed < 2:
                self.t1_nav_extra()
            else:
                self.t1_nav_extra_final()
        elif self.state == State.T1_EXTRA_FORWARD_NAVIGATING:
            time.sleep(5.0)
            self.state = State.T1_FINDING_NEXT_GATE
        elif self.state == State.T1_FINAL_EXTRA_FORWARD_NAVIGATING:
            self.state = State.START
        elif self.state == State.T2_ENTERING:
            self.state = State.START
        elif self.state == State.T3_SEARCH_TRANSLATING:
            self.state = State.T3_CODE_SEARCH
        elif self.state == State.T3_CENTERING:
            self.state = State.T3_CODE_SEARCH
        elif self.state == State.T4_NAVIGATING_TO_GATE_MIDPOINT:
            time.sleep(2.0)
            self.t4_nav_forward()
        elif self.state == State.T4_EXTRA_FORWARD_NAV:
            self.state = State.T4_CHECKING_FOR_LOOP_BUOY
        elif self.state == State.T4_LOOPING:
            self.t4_final_nav_forward() 
        elif self.state == State.T4_FINAL_EXTRA_FORWARD_NAV:
            self.state = State.START
        elif self.state == State.T6_BACKING_UP:
            self.shoot_balls()
    
    def t1_nav_extra(self):
        self.state = State.T1_EXTRA_FORWARD_NAVIGATING
        self.trans_pub.publish(Point(x=self.get_parameter('t1_extra_forward_nav').value))
    
    def t1_nav_extra_final(self):
        self.state = State.T1_FINAL_EXTRA_FORWARD_NAVIGATING
        self.trans_pub.publish(Point(x=self.get_parameter('t1_final_extra_forward_nav').value))
    
    def execute(self):
        self.get_logger().info(str(self.state))
        self.timeout_secs += 1
        self.t6_backup_time += 1
        self.t6_rotate_time += 1

        if self.state.value >= 8:
            self.docking_timeout_secs += 1
        
        if (self.state == State.T6_BACKING_UP 
            and self.t6_backup_time > self.get_parameter('timeouts.t6_back_up').value):
            self.get_logger().info('Timed out backing up')
            self.trans_pub.publish(Point())
            self.shoot_balls()
            return
        
        if (self.state == State.T6_ROTATING 
            and self.t6_rotate_time > self.get_parameter('timeouts.t6_rotate').value):
            self.get_logger().info('Timed out rotating')
            self.t6_back_up()
            return

        if self.state == State.START:
            self.get_logger().info(f'On task {self.curr_task+2} of {len(self.task_nums)}')
            self.start_next_task()
        elif self.state == State.T1_FINDING_NEXT_GATE:
            self.t1_find_next_gate()
        elif self.state == State.T3_CODE_SEARCH:
            self.t3_code_search()
        elif self.state == State.T4_FINDING_GATE:
            self.t4_find_gate()
        elif self.state == State.T4_CHECKING_FOR_LOOP_BUOY:
            self.t4_find_loop_buoy()
        elif self.state == State.T6_ENTERING_BALL_TARGET:
            self.t6_enter_ball_target()
    
    def start_next_task(self):
        if self.curr_task + 1 == len(self.task_nums):
            self.state = State.COMPLETE
            return

        self.curr_task += 1

        self.state = State.TASK_WAYPOINT_NAVIGATING

        msg = TaskWaypointNav.Goal()
        msg.task_num = self.task_nums[self.curr_task]

        self.nav_req = self.nav_client.send_goal_async(msg)

        self.nav_req.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.nav_result = goal_handle.get_result_async()
        self.nav_result.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        time.sleep(2.0)
        self.post_waypoint_nav_op()
    
    def post_waypoint_nav_op(self):
        self.timeout_secs = 0
        self.docking_timeout_secs = 0
        if self.task_nums[self.curr_task] == 1:
            if self.get_parameter('t1_auto').value:
                self.state = State.T1_FINDING_NEXT_GATE
            else:
                self.state = State.START
        elif self.task_nums[self.curr_task] == 2:
            if self.get_parameter('t2_auto').value:
                self.t2_auto_enter()
            else:
                self.state = State.START
        elif self.task_nums[self.curr_task] == 3:
            if self.get_parameter('t3_auto').value:
                self.state = State.T3_CODE_SEARCH
            else:
                self.state = State.START
        elif self.task_nums[self.curr_task] == 4:
            if self.get_parameter('t4_auto').value:
                self.state = State.T4_FINDING_GATE
            else:
                self.state = State.START
        elif self.task_nums[self.curr_task] == 6:
            if self.get_parameter('t6_auto').value:
                self.state = State.T6_ENTERING_BALL_TARGET
            else:
                self.state = State.START
    
    def t6_enter_ball_target(self):
        if self.approach_req is not None:
            return

        msg = ApproachTarget.Goal()
        msg.approach_dist = self.get_parameter('t6_approach_dist').value
        msg.scan_width = self.get_parameter('t6_approach_scan_width').value
        msg.timeout = float(self.get_parameter('timeouts.t6_approach').value)

        self.approach_req = self.approach_client.send_goal_async(msg)
        self.approach_req.add_done_callback(self.t6_approach_response_callback)
    
    def t6_approach_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.approach_result = goal_handle.get_result_async()
        self.approach_result.add_done_callback(self.t6_approach_result_callback)
    
    def t6_approach_result_callback(self, future):
        self.approach_req = None
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        time.sleep(3.0)
        self.t6_rotate()
    
    def t6_rotate(self):
        self.t6_rotate_time = 0
        self.state = State.T6_ROTATING
        msg = Rotate.Request()
        msg.goal = Vector3(z=math.pi)

        self.rotate_call = self.rotate_client.call_async(msg)
        self.rotate_call.add_done_callback(self.t6_rotate_response)
    
    def t6_rotate_response(self, future):
        result:Rotate.Response = future.result()
        self.get_logger().info(f'rotate response: {result}')
        self.t6_back_up()
    
    def t6_back_up(self):
        self.t6_backup_time = 0
        self.state = State.T6_BACKING_UP
        self.trans_pub.publish(Point(x=-self.get_parameter('t6_back_up_dist').value))
    
    def shoot_balls(self):
        self.state = State.T6_SHOOTING

        msg = ShootBalls.Goal()

        self.ball_shooter_req = self.ball_shooter_client.send_goal_async(msg) 

        self.ball_shooter_req.add_done_callback(self.t6_ball_shooter_response_callback)
    
    def t6_ball_shooter_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.ball_shooter_result = goal_handle.get_result_async()
        self.ball_shooter_result.add_done_callback(self.t6_ball_shooter_result_callback)
    
    def t6_ball_shooter_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.state = State.START
    
    def t4_nav_forward(self):
        self.state = State.T4_EXTRA_FORWARD_NAV
        self.trans_pub.publish(Point(x=self.get_parameter('t4_gate_extra_forward_nav').value))
    
    def t4_final_nav_forward(self):
        self.state = State.T4_FINAL_EXTRA_FORWARD_NAV
        self.trans_pub.publish(Point(x=self.get_parameter('t4_final_extra_forward_nav').value))
    
    def t4_find_gate(self):
        if self.channel_call is not None:
            return
        
        if self.timeout_secs > self.get_parameter('timeouts.t4_gate').value:
            self.get_logger().info('Timed out of finding gate')
            self.state = State.T4_NAVIGATING_TO_GATE_MIDPOINT
            point = Point(x=self.get_parameter('timeout_responses.t4_gate_trans').value)
            self.trans_pub.publish(point)
            self.t4_gate_midpoint = PoseStamped()
            self.t4_gate_midpoint.pose.position.x = self.robot_pose.pose.position.x
            self.t4_gate_midpoint.pose.position.y = self.robot_pose.pose.position.y
            self.t4_gate_midpoint.pose.orientation.x = self.robot_pose.pose.orientation.x
            self.t4_gate_midpoint.pose.orientation.y = self.robot_pose.pose.orientation.y
            self.t4_gate_midpoint.pose.orientation.z = self.robot_pose.pose.orientation.z
            self.t4_gate_midpoint.pose.orientation.w = self.robot_pose.pose.orientation.w
            return

        req = Channel.Request()
        req.left_color = 'red'
        req.right_color = 'green'
        req.use_lidar = True
        req.use_camera = False
        req.max_dist_from_usv = self.get_parameter('t4_gate_max_buoy_dist').value

        self.channel_call = self.channel_client.call_async(req)
        self.channel_call.add_done_callback(self.t4_channel_response)
    
    def t4_channel_response(self, future):
        result:Channel.Response = future.result()
        self.get_logger().info(f'channel response: {result}')

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

        self.state = State.T4_NAVIGATING_TO_GATE_MIDPOINT
        self.t4_gate_midpoint = mid
        self.channel_call = None
        self.path_pub.publish(path)
    
    def t4_find_loop_buoy(self):
        if self.channel_call is not None:
            return

        if self.timeout_secs > self.get_parameter('timeouts.t4_loop').value:
            self.get_logger().info('Timed out of finding loop')
            self.t4_gate_midpoint.pose.orientation = self.find_reverse_orientation(self.t4_gate_midpoint.pose.orientation)
            path = Path()
            path.poses.append(self.t4_gate_midpoint)
            self.state = State.T4_LOOPING
            self.path_pub.publish(path)
            return

        req = Channel.Request()
        req.left_color = 'blue'
        req.right_color = 'blue'
        req.use_lidar = True
        req.use_camera = False
        req.max_dist_from_usv = self.get_parameter('t4_loop_max_buoy_dist').value

        self.get_logger().info('sending channel request')
        self.channel_call = self.channel_client.call_async(req)
        self.channel_call.add_done_callback(self.t4_loop_buoy_response)
    
    def t4_loop_buoy_response(self, future):
        result:Channel.Response = future.result()
        self.get_logger().info(f'channel response: {result}')

        null_point = Point(x=0.0,y=0.0,z=0.0)

        self.channel_call = None

        left_ps = point_to_pose_stamped(result.left)
        right_ps = point_to_pose_stamped(result.right)
        
        if ((result.left == null_point) 
            and (result.right == null_point)):
            self.get_logger().info('No loop buoy found')
            return
        
        if result.left == null_point:
            pose = right_ps
        elif result.right == null_point:
            pose = left_ps
        elif (distance_pose_stamped(left_ps, self.robot_pose)
            < distance_pose_stamped(right_ps, self.robot_pose)):
            pose = left_ps
        else:
            pose = right_ps
        
        self.get_logger().info(f'pose: {pose}')
        
        path = LoopingBuoy.find_path_around_buoy(self.robot_pose, pose,
            looping_radius=self.get_parameter('t4_looping_radius').value)
        
        self.t4_gate_midpoint.pose.orientation = path.poses[-1].pose.orientation
        path.poses.append(self.t4_gate_midpoint)

        self.state = State.T4_LOOPING
        self.channel_call = None
        self.path_pub.publish(path)

    def find_reverse_orientation(self, o:Quaternion):
        e = tf_transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        q = tf_transformations.quaternion_from_euler(e[0], e[1], e[2] + math.pi)
        return Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
    
    def t3_code_search(self):
        if self.code_pos_req is not None:
            return
        
        if self.docking_timeout_secs > self.get_parameter('timeouts.t3_code_search').value:
            self.get_logger().info('Timed out of code search')
            self.t3_dock()
            return
        if self.get_parameter('t3_skip_line_up').value:
            self.t3_dock()
            return

        msg = DockCodesCameraPos.Request()

        self.code_pos_req = self.code_pos_client.call_async(msg)        
        self.code_pos_req.add_done_callback(self.t3_code_pos_callback)
    
    def t3_code_pos_callback(self, future):
        result = future.result()

        self.code_pos_req = None

        self.get_logger().info(f'result: {result}')

        if len(result.red) == 0:
            self.get_logger().info(f'No dock code positions sent')
            return
        
        targetX = -1
        othersX = 0
        othersCount = 0

        if result.red[0] != -1 and result.red[1] != -1:
            mid = (result.red[0] + result.red[1]) // 2
            if self.t3_target_color == 'red':
                targetX = mid
            else:
                othersX += mid
                othersCount += 1
        
        if result.blue[0] != -1 and result.blue[1] != -1:
            mid = (result.blue[0] + result.blue[1]) // 2
            if self.t3_target_color == 'blue':
                targetX = mid
            else:
                othersX += mid
                othersCount += 1
        
        if result.green[0] != -1 and result.green[1] != -1:
            mid = (result.green[0] + result.green[1]) // 2
            if self.t3_target_color == 'green':
                targetX = mid
            else:
                othersX += mid
                othersCount += 1
        
        if othersCount > 0:
            othersX //= othersCount
        
        if self.state == State.T3_CENTERING and targetX == -1:
            self.get_logger().info('Centering but code not found.')
            return
        
        if targetX > -1:
            self.t3_center_translate(targetX, result.image_width)
            return
        
        if othersCount == 0:
            self.get_logger().info('No code found in search.')
            return
        
        self.t3_search_translate(othersX, result.image_width)
    
    def t3_center_translate(self, targetX, image_width):
        msg = Point()

        self.state = State.T3_CENTERING

        movement_vals = self.get_parameter('t3_centering_movement_values').value

        loc = targetX / image_width

        for i, tranch in enumerate(self.get_parameter('t3_centering_movement_tranches').value):
            if not loc < tranch: continue

            if movement_vals[i] == 0.0:
                self.t3_dock()
                return
            
            self.get_logger().info(f'translating: {movement_vals[i]}')
            msg.y = movement_vals[i]
            break
        else:
            self.get_logger().info(f'translating: {movement_vals[-1]}')
            msg.y = movement_vals[-1]

        self.trans_pub.publish(msg)
    
    def t3_search_translate(self, othersX, image_width):
        msg = Point()

        self.state = State.T3_SEARCH_TRANSLATING

        if othersX < image_width * .5:
            msg.y = self.get_parameter('t3_search_movement_value').value
        else:
            msg.y = -self.get_parameter('t3_search_movement_value').value

        self.trans_pub.publish(msg)
    
    def t3_dock(self):
        self.state = State.T3_DOCKING
        self.docking_timeout_secs = 0

        msg = ApproachTarget.Goal()
        msg.approach_dist = self.get_parameter('t3_approach_dist').value
        msg.scan_width = self.get_parameter('t3_approach_scan_width').value
        msg.timeout = float(self.get_parameter('timeouts.t3_approach').value)

        self.approach_req = self.approach_client.send_goal_async(msg)
        self.approach_req.add_done_callback(self.t3_approach_response_callback)
    
    def t3_approach_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.approach_result = goal_handle.get_result_async()
        self.approach_result.add_done_callback(self.t3_approach_result_callback)
    
    def t3_approach_result_callback(self, future):
        self.approach_req = None
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.state = State.START

    def t2_auto_enter(self):
        self.state = State.T2_ENTERING
        self.trans_pub.publish(Point(
            x=self.get_parameter('t2_trans_x').value,
            y=self.get_parameter('t2_trans_y').value
        ))
    
    def t1_find_next_gate(self):

        if self.robot_pose is None:
            return
        if self.channel_call is not None:
            return
        
        if self.timeout_secs > self.get_parameter('timeouts.t1_find_next_gate').value:
            self.get_logger().info('Hit Timeout')
            self.state = State.T1_GATE_NAVIGATING
            self.trans_pub.publish(Point(x=self.get_parameter('timeout_responses.t1_find_next_gate_trans').value))
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

        self.state = State.T1_GATE_NAVIGATING
        self.channel_call = None
        self.channels_completed += 1
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)

    node = FinalsNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
