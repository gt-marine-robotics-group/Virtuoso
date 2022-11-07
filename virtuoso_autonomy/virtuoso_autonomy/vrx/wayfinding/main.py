import rclpy
from rclpy.node import Node
from virtuoso_msgs.msg import Task
from geographic_msgs.msg import GeoPath
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from robot_localization.srv import FromLL

class Wayfinding(Node):

    def __init__(self):
        super().__init__('wayfinding')

        self.path = Path()
        self.path_len = None

        self.geo_path = None
        self.geo_pose_count = 0

        self.task_info_sub = self.create_subscription(Task, '/vrx/task/info', self.task_info_callback, 10)
        self.path_sub = self.create_subscription(GeoPath, '/vrx/wayfinding/waypoints', self.path_callback, 10)

        self.path_pub = self.create_publisher(Path, '/virtuoso_navigation/set_path', 10)

        self.state = 'initial'
        self.path_sent = False

        self.fromLL_cli = self.create_client(FromLL, '/fromLL')

    def task_info_callback(self, msg:Task):

        if (msg.name == 'wayfinding'): self.update_state(msg)
    
    def update_state(self, msg:Task):
        self.state = msg.state

        if (self.state == 'running'): self.run()
    
    def run(self):

        if (self.path_sent): return
        if (self.path_len is None): return
        if (self.path_len > len(self.path.poses)): return

        self.path_sent = True
        self.path_pub.publish(self.path)

    def path_callback(self, msg:GeoPath):
        
        self.geo_path = msg
        self.path_len = len(msg.poses)

        def ll_callback(future):
            point = future.result().map_point
            pose_stamped = PoseStamped()
            pose = Pose()
            pose.position = point
            pose.orientation = self.geo_path.poses[self.geo_pose_count].pose.orientation
            pose_stamped.pose = pose
            self.path.poses.append(pose_stamped)
            
            self.geo_pose_count += 1
            if (self.geo_pose_count >= len(self.geo_path.poses)): return
            ll = self.create_ll_future()
            ll.add_done_callback(ll_callback)

        ll = self.create_ll_future()
        ll.add_done_callback(ll_callback)
    
    def create_ll_future(self):
        self.req = FromLL.Request()
        self.req.ll_point = self.geo_path.poses[self.geo_pose_count].pose.position
        return self.fromLL_cli.call_async(self.req)


def main(args=None):
    
    rclpy.init(args=args)

    node = Wayfinding()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()