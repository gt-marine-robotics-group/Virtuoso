import rclpy
from rclpy.node import Node
from virtuoso_msgs.msg import Task
from geometry_msgs.msg import PoseStamped, Pose
from geographic_msgs.msg import GeoPoseStamped
from robot_localization.srv import FromLL

class StationKeeping(Node):

    def __init__(self):
        super().__init__('station_keeping')

        self.received_goal = None
        self.prev_goal = None
        self.goal_to_send = None

        self.task_info_sub = self.create_subscription(Task, '/vrx/task/info', self.task_info_callback, 10)
        self.goal_sub = self.create_subscription(GeoPoseStamped, '/vrx/station_keeping/goal', self.goal_callback, 10)

        self.goal_pub = self.create_publisher(PoseStamped, '/virtuoso_navigation/set_goal', 10)

        self.is_task1 = False
        self.state = 'initial'

        self.fromLL_cli = self.create_client(FromLL, '/fromLL')
    
    def task_info_callback(self, msg:Task):

        if (msg.name == 'station_keeping'): self.update_state(msg)

    def update_state(self, msg:Task):
        self.state = msg.state

        if (self.state == 'running'): self.run()
    
    def goal_callback(self, msg:GeoPoseStamped):
        self.received_goal = msg
    
    def run(self):

        # For future, may want to see if it is possible to create path and then 
        # start navigation later. We get the path in the "ready" state and can
        # begin moving there in the "running" state.

        if (self.received_goal is None): return
        
        # Check to make sure we're not passing a redundant goal
        if (self.prev_goal is None):
            pass
        elif (self.received_goal.pose.position.latitude == self.prev_goal.pose.position.latitude 
            and self.received_goal.pose.position.longitude == self.prev_goal.pose.position.longitude):
            return

        self.prev_goal = self.received_goal 

        def ll_callback(future):
            point = future.result().map_point
            pose_stamped = PoseStamped()
            pose = Pose()
            pose.position = point
            pose_stamped.pose = pose
            self.goal_pub.publish(pose_stamped)

        self.req = FromLL.Request()
        self.req.ll_point = self.received_goal.pose.position
        self.get_logger().info(str(self.req))
        dest = self.fromLL_cli.call_async(self.req)
        dest.add_done_callback(ll_callback)



def main(args=None):
    
    rclpy.init(args=args)

    node = StationKeeping()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()