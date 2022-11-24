import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import tf_transformations
import numpy

class CmdVelGenerator(Node):

    def __init__(self):
        super().__init__('controller_cmd_vel_generator')
        
        self.state_estimate = Odometry()
        self.destination = Pose()
        self.received_path = False
        self.next_waypoint = Pose()
        self.second_waypoint = Pose()
        self.nav2_path = Path()
        self.hold_final_orient = False
        
        self.path_subscriber = self.create_subscription(
            Path,
            '/navigation/plan',
            self.path_callback,
            10)  
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odometry_callback,
            10)   
        self.hold_final_orientation_sub = self.create_subscription(
            Bool, '/controller/is_translation', self.hold_final_orient_callback, 10)
            
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def hold_final_orient_callback(self, msg:Bool):
        self.hold_final_orient = msg.data
        
    def path_callback(self, msg:Path):
        self.destination = msg.poses[-1].pose 
        self.nav2_path = msg
        self.received_path = True
       
    def timer_callback(self):
        if not self.received_path:
            return

        dest_x = self.destination.position.x
        dest_y = self.destination.position.y

        self_x = self.state_estimate.pose.pose.position.x
        self_y = self.state_estimate.pose.pose.position.y
        
        dist_to_target = ((self_x - dest_x)**2 + (self_y - dest_y)**2)**(1/2)
        
        min_pose_distance = 100000.0
        closest_pose = 0
        for i in range(0,len(self.nav2_path.poses)):

            next_pose_distance = (
                (self.nav2_path.poses[i].pose.position.x - self_x)**2 + 
                (self.nav2_path.poses[i].pose.position.y - self_y)**2
            )**(1/2)

            if (next_pose_distance < min_pose_distance):
                closest_pose = i
                min_pose_distance = next_pose_distance
                        
        next_x = self.nav2_path.poses[closest_pose].pose.position.x
        next_y = self.nav2_path.poses[closest_pose].pose.position.y
        if(len(self.nav2_path.poses) > closest_pose + 1):
            second_x = self.nav2_path.poses[closest_pose + 1].pose.position.x
            second_y = self.nav2_path.poses[closest_pose + 1].pose.position.y
        else:
            second_x = next_x
            second_y = next_y
            
        if(second_x != next_x or second_y != next_y):
            min_pose_distance = abs(
                (second_x - next_x)*(next_y - self_y) - (next_x - self_x)*(second_y - next_y)
            ) / ((second_x - next_x)**2 + (second_y - next_y)**2)**(1/2)    

        q = [self.state_estimate.pose.pose.orientation.x, 
            self.state_estimate.pose.pose.orientation.y, 
            self.state_estimate.pose.pose.orientation.z, 
            self.state_estimate.pose.pose.orientation.w]
        q_inv = q.copy()
        q_inv[0] = -q_inv[0]
        q_inv[1] = -q_inv[1]
        q_inv[2] = -q_inv[2]  
        
        
        vel_parallel = [float(second_x - next_x), float(second_y - next_y), 0.0, 0.0]

        vel_parallel = tf_transformations.quaternion_multiply(q_inv, vel_parallel)
        vel_parallel2 = tf_transformations.quaternion_multiply(vel_parallel, q)      
        vel_parallel = [vel_parallel2[0], vel_parallel2[1], vel_parallel2[2], vel_parallel2[3]]       

        vel_angle = numpy.arctan2(vel_parallel[0], vel_parallel[1])
        
        vel_parallel_speed = 2.0
        if(dist_to_target < 6.0):
            vel_parallel_speed = dist_to_target/3.0
        if(vel_parallel_speed < 0.3):
            vel_parallel_speed = 0.3
        
        vel_parallel_speed = vel_parallel_speed - vel_parallel_speed*min_pose_distance/3.0
        
        if(vel_parallel_speed < 0.3):
            vel_parallel_speed = 0.3
        
        vel_parallel_mag = vel_parallel_speed/((vel_parallel[0])**2 + (vel_parallel[1])**2)**(1/2)
        
        vel_parallel[0] = vel_parallel[0]*vel_parallel_mag
        vel_parallel[1] = vel_parallel[1]*vel_parallel_mag
        
        if(numpy.isnan(vel_parallel[0])):
            vel_parallel[0] = 0.0
        if(numpy.isnan(vel_parallel[1])):
            vel_parallel[1] = 0.0
        
        closest_x = 0.0
        closest_y = 0.0
        
        if(second_x != next_x and second_y != next_y):
            m1 = (second_y - next_y)/(second_x - next_x)
            m2 = -1/m1
            closest_x = (m2*self_x - m1*next_x + next_y - self_y)/(m2 - m1)
            closest_y = m2*(closest_x - self_x) + self_y
        elif(second_x != next_x and second_y == next_y):
            closest_y = second_y
            closest_x = self_x
        elif(second_x == next_x and second_y != next_x):
            closest_x = second_x
            closest_y = self_y
        else:
            closest_x = next_x
            closest_y = next_y
        
        vel_towards = [float(closest_x - self_x), float(closest_y - self_y), 0.0, 0.0]
        vel_towards = tf_transformations.quaternion_multiply(q_inv, vel_towards)
        vel_towards2 = tf_transformations.quaternion_multiply(vel_towards, q)      
        vel_towards = [vel_towards2[0], vel_towards2[1], vel_towards2[2], vel_towards2[3]]       
        
        speed_towards = min_pose_distance/2.0
        if(speed_towards > 2.0):
            speed_towards = 2.0
        if(speed_towards < 0.05):
            speed_towards = 0.05
        
        if(speed_towards < 0.05):
            speed_towards = 0.05
                    
        vel_towards_mag = speed_towards/((vel_towards[0])**2 + (vel_towards[1])**2)**(1/2)
                
        vel_towards[0] = vel_towards[0]*vel_towards_mag
        vel_towards[1] = vel_towards[1]*vel_towards_mag      
                        
        vel_to_command = Twist()
        vel_to_command.linear.x = vel_parallel[0] + vel_towards[0]
        vel_to_command.linear.y = vel_parallel[1] + vel_towards[1]
        
        vel_angle = numpy.arctan2(vel_to_command.linear.x, vel_to_command.linear.y)
        
        if(not self.hold_final_orient):
            vel_to_command.linear.x = (vel_to_command.linear.x 
                - vel_to_command.linear.x*abs(vel_angle)/numpy.pi/2)
            vel_to_command.linear.y = (vel_to_command.linear.y 
                - vel_to_command.linear.y*abs(vel_angle)/numpy.pi/2)
        
        self.cmd_vel_publisher.publish(vel_to_command)
  
    def odometry_callback(self, msg:Odometry):
        self.state_estimate = msg       

        
def main(args=None):
    rclpy.init(args=args)

    cmd_vel_generator = CmdVelGenerator()

    rclpy.spin(cmd_vel_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_vel_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
