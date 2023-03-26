import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from virtuoso_msgs.action import TaskWaypointNav
from .semis_states import State

class SemisNode(Node):

    def __init__(self):
        super().__init__('autonomy_semis')

        self.declare_parameters(namespace='', parameters=[
            ('task_nums', []),
            ('docking_num', -1),
            ('ball_shooter_num', -1),
            ('docking_secs', 1)
        ])

        self.task_nums = self.get_parameter('task_nums').value

        self.curr_task = -1

        self.nav_client = ActionClient(self, TaskWaypointNav, 'task_waypoint_nav')
        self.nav_req = None
        self.nav_result = None

        self.curr_docking_time = 0

        self.state = State.START

        self.create_timer(1.0, self.execute)
    
    def execute(self):
        self.get_logger().info(str(self.state))
        self.get_logger().info(f'On task {self.curr_task+1} of {len(self.task_nums)}')

        if len(self.task_nums) == 0:
            self.state = State.COMPLETE

        if self.state == State.START:
            self.start_next_task()
        elif self.state == State.DOCKING_STOP:
            if self.curr_docking_time >= self.get_parameter('docking_secs').value:
                self.start_next_task()
            else:
                self.curr_docking_time += 1
    
    def start_next_task(self):
        self.get_logger().info('Starting next task')
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
        task_num = self.task_nums[self.curr_task]

        if task_num == self.get_parameter('docking_num').value:
            self.state = State.DOCKING_STOP
        elif task_num == self.get_parameter('ball_shooter_num').value:
            self.state = State.BALL_SHOOTING
        else:
            self.start_next_task()


def main(args=None):
    rclpy.init(args=args)

    node = SemisNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()