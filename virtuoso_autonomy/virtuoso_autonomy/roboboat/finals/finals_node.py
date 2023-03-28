import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path, Odometry
from virtuoso_msgs.srv import Channel, DockCodesCameraPos
from virtuoso_msgs.action import TaskWaypointNav
from .finals_states import State
from ...utils.channel_nav.channel_nav import ChannelNavigation
from ...utils.geometry_conversions import point_to_pose_stamped
import time

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

            ('timeouts.t1_find_next_gate', 0),
            ('timeouts.t3_code_search', 0),

            ('timeout_responses.t1_find_next_gate_trans', 0.0)
        ])

        self.t3_target_color = self.get_parameter('t3_target_color').value

        self.task_nums = self.get_parameter('task_nums').value

        self.curr_task = 2 # TEMP -1

        self.nav_client = ActionClient(self, TaskWaypointNav, 'task_waypoint_nav')
        self.nav_req = None
        self.nav_result = None

        self.path_pub = self.create_publisher(Path, '/navigation/set_path', 10)
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

        self.timeout_secs = 0
        self.docking_timeout_secs = 0

        self.state = State.T3_CODE_SEARCH # TEMP State.START

        self.channels_completed = 0

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
    
    def t1_nav_extra(self):
        self.state = State.T1_EXTRA_FORWARD_NAVIGATING
        self.trans_pub.publish(Point(x=self.get_parameter('t1_extra_forward_nav').value))
    
    def t1_nav_extra_final(self):
        self.state = State.T1_FINAL_EXTRA_FORWARD_NAVIGATING
        self.trans_pub.publish(Point(x=self.get_parameter('t1_final_extra_forward_nav').value))
    
    def execute(self):
        self.get_logger().info(str(self.state))
        self.timeout_secs += 1

        if self.state.value >= 8:
            self.docking_timeout_secs += 1

        if self.state == State.START:
            self.get_logger().info(f'On task {self.curr_task+2} of {len(self.task_nums)}')
            self.start_next_task()
        elif self.state == State.T1_FINDING_NEXT_GATE:
            self.t1_find_next_gate()
        elif self.state == State.T3_CODE_SEARCH:
            self.t3_code_search()
        elif self.state == State.T3_DOCKING:
            pass
    
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
        self.post_waypoint_nav_op()
    
    def post_waypoint_nav_op(self):
        self.timeout_secs = 0
        self.docking_timeout_secs += 1
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
    
    def t3_code_search(self):
        if self.code_pos_req is not None:
            return
        
        if self.timeout_secs > self.get_parameter('timeouts.t3_code_search').value:
            self.state = State.T3_DOCKING
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
        self.docking_timeout_secs = 0

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