import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
from robot_localization.srv import FromLL
from ros_gz_interfaces.msg import ParamVec

from virtuoso_autonomy.utils.task_info import get_state


import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class StationKeepingNode(Node):

    def __init__(self):
        super().__init__('station_keeping')

        self.state = 'initial'
        self.goal:PoseStamped = None
        self.executing = False
        self.transformed_goal = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.task_info_sub = self.create_subscription(ParamVec, '/vrx/task/info', 
            self.task_info_callback, 10)
        
        self.goal_sub = self.create_subscription(PoseStamped, '/vrx/stationkeeping/goal',
            self.goal_callback, 10)

        self.fromLL_cli = self.create_client(FromLL, '/fromLL')

        self.path_pub = self.create_publisher(Path, '/navigation/set_waypoints', 10)
        
        self.create_timer(1.0, self.execute)
    
    def task_info_callback(self, msg:ParamVec):
        # self.get_logger().info(str(msg.params))
        # self.get_logger().info(f'state: {get_state(msg)}')
        self.state = get_state(msg)
    
    def goal_callback(self, msg:PoseStamped):
        self.goal = msg
    
    def execute(self):
        self.get_logger().info(f'trans goal: {self.transformed_goal}')
    
        if self.executing:
            return

        if self.goal is None:
            return
        
        if not self.transformed_goal:
            self.get_logger().info('transforming goal')
            self.executing = True
            self.transform_goal()
            return
        
        self.executing = True

        path = Path()
        path.poses.append(self.transformed_goal)
        self.path_pub.publish(path)
    
    def transform_goal(self):
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
            point = future.result().map_point
            pose_stamped = PoseStamped()
            pose = Pose()
            pose.position = point

            q_goal = [self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z, self.goal.pose.orientation.w]
            
            q_utm_to_odom = [self.quatUTMtoODOM.x, self.quatUTMtoODOM.y, self.quatUTMtoODOM.z, self.quatUTMtoODOM.w]
            q_utm_to_odom = [0, 0, 0, 1]            
            q_final = tf_transformations.quaternion_multiply(q_utm_to_odom, q_goal)
            pose.orientation.x = q_final[0]
            pose.orientation.y = q_final[1]
            pose.orientation.z = q_final[2]
            pose.orientation.w = q_final[3]

            pose_stamped.pose = pose
            # self.goal_pub.publish(pose_stamped)
            self.transformed_goal = pose_stamped
            self.executing = False

        self.req = FromLL.Request()
        # self.req.ll_point = self.goal.pose.position
        self.req.ll_point = GeoPoint()
        self.req.ll_point.longitude = self.goal.pose.position.y
        self.req.ll_point.latitude = self.goal.pose.position.x
        self.req.ll_point.altitude = self.goal.pose.position.z
        self.get_logger().info(str(self.req))
        dest = self.fromLL_cli.call_async(self.req)
        dest.add_done_callback(ll_callback)



# class StationKeeping(Node):

#     def __init__(self):
#         super().__init__('station_keeping')

#         self.received_goal = None
#         self.prev_goal = None
#         self.goal_to_send = None

#         self.task_info_sub = self.create_subscription(Task, '/vrx/task/info', self.task_info_callback, 10)
#         self.goal_sub = self.create_subscription(GeoPoseStamped, '/vrx/station_keeping/goal', self.goal_callback, 10)

#         self.goal_pub = self.create_publisher(PoseStamped, '/virtuoso_navigation/set_goal', 10)

#         self.is_task1 = False
#         self.state = 'initial'

#         self.fromLL_cli = self.create_client(FromLL, '/fromLL')

        
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        

    
#     def task_info_callback(self, msg:Task):

#         if (msg.name == 'station_keeping'): self.update_state(msg)

#     def update_state(self, msg:Task):
#         self.state = msg.state

#         if (self.state == 'running'): self.run()
    
#     def goal_callback(self, msg:GeoPoseStamped):
#         self.received_goal = msg
    
#     def run(self):

#         # For future, may want to see if it is possible to create path and then 
#         # start navigation later. We get the path in the "ready" state and can
#         # begin moving there in the "running" state.

#         if (self.received_goal is None): return
        
#         # Check to make sure we're not passing a redundant goal
#         if (self.prev_goal is None):
#             pass
#         elif (self.received_goal.pose.position.latitude == self.prev_goal.pose.position.latitude 
#             and self.received_goal.pose.position.longitude == self.prev_goal.pose.position.longitude):
#             return

#         self.prev_goal = self.received_goal 

        
#         from_frame_rel = 'utm'
#         to_frame_rel = 'odom'
        
#         try:
#              now = rclpy.time.Time()
#              trans = self.tf_buffer.lookup_transform(
#                         to_frame_rel,
#                         from_frame_rel,
#                         now)
#         except TransformException as ex:
#              self.get_logger().info(
#                         f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
#              return
#         self.quatUTMtoODOM = trans.transform.rotation


#         def ll_callback(future):
#             point = future.result().map_point
#             pose_stamped = PoseStamped()
#             pose = Pose()
#             pose.position = point

#             q_goal = [self.received_goal.pose.orientation.x, self.received_goal.pose.orientation.y, self.received_goal.pose.orientation.z, self.received_goal.pose.orientation.w]
            
#             q_utm_to_odom = [self.quatUTMtoODOM.x, self.quatUTMtoODOM.y, self.quatUTMtoODOM.z, self.quatUTMtoODOM.w]
#             q_utm_to_odom = [0, 0, 0, 1]            
#             q_final = tf_transformations.quaternion_multiply(q_utm_to_odom, q_goal)
#             pose.orientation.x = q_final[0]
#             pose.orientation.y = q_final[1]
#             pose.orientation.z = q_final[2]
#             pose.orientation.w = q_final[3]

#             pose_stamped.pose = pose
#             self.goal_pub.publish(pose_stamped)

#         self.req = FromLL.Request()
#         self.req.ll_point = self.received_goal.pose.position
#         self.get_logger().info(str(self.req))
#         dest = self.fromLL_cli.call_async(self.req)
#         dest.add_done_callback(ll_callback)



def main(args=None):
    
    rclpy.init(args=args)

    node = StationKeepingNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

