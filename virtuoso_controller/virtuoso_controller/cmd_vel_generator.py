import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
import tf_transformations

import numpy

class cmdVelGenerator(Node):

    def __init__(self):
        super().__init__('cmd_vel_generator')
        
        self.stateEstimate = Odometry()
        self.destination = Pose()
        self.receivedPath = False
        self.nextWaypoint = Pose()
        self.secondWaypoint = Pose()
        self.nav2Path = Path()
        
        self.path_subscriber = self.create_subscription(
            Path,
            '/transformed_global_plan',
            self.path_callback,
            10)  
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odometry_callback,
            10)   

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def path_callback(self, msg):
        self.destination = msg.poses[-1].pose 
        #self.nextWaypoint = msg.poses[0].pose
        #if(len(msg.poses) > 1):
         #    self.secondWaypoint = msg.poses[1].pose
        #else:
         #    self.secondWaypoint = msg.poses[0].pose
        self.nav2Path = msg
        self.receivedPath = True
       
    def timer_callback(self):
        if(self.receivedPath):
             destX = self.destination.position.x
             destY = self.destination.position.y
        
             selfX = self.stateEstimate.pose.pose.position.x
             selfY = self.stateEstimate.pose.pose.position.y
             
             distToTarget = ((selfX - destX)**2 + (selfY - destY)**2)**(1/2)
             
             minPoseDistance = 100000.0
             closestPose = 0
             for i in range(0,len(self.nav2Path.poses)):
                  nextPoseDistance = ((self.nav2Path.poses[i].pose.position.x - selfX)**2 + (self.nav2Path.poses[i].pose.position.y - selfY)**2)**(1/2)
                  if (nextPoseDistance < minPoseDistance):
                      closestPose = i
                      minPoseDistance = nextPoseDistance
             #if(len(self.nav2Path.poses) > i+1):
             #     closestPose = i + 1   


                               
             nextX = self.nav2Path.poses[closestPose].pose.position.x
             nextY = self.nav2Path.poses[closestPose].pose.position.y
             if(len(self.nav2Path.poses) > closestPose+1):
                  secondX = self.nav2Path.poses[closestPose + 1].pose.position.x
                  secondY = self.nav2Path.poses[closestPose + 1].pose.position.y
             else:
                 secondX = nextX
                 secondY = nextY
                 
             if(secondX != nextX or secondY != nextY):
                 minPoseDistance = abs((secondX - nextX)*(nextY - selfY) - (nextX - selfX)*(secondY - nextY))/((secondX - nextX)**2 + (secondY - nextY)**2)**(1/2)    
             self.get_logger().info('minPoseDistance: ' + str(minPoseDistance))                          
             q = [self.stateEstimate.pose.pose.orientation.x, self.stateEstimate.pose.pose.orientation.y, self.stateEstimate.pose.pose.orientation.z, self.stateEstimate.pose.pose.orientation.w]
             q_inv = q.copy()
             q_inv[0] = -q_inv[0]
             q_inv[1] = -q_inv[1]
             q_inv[2] = -q_inv[2]  
             
             
             vel_parallel = [float(secondX - nextX), float(secondY - nextY), 0.0, 0.0]

             vel_parallel = tf_transformations.quaternion_multiply(q_inv, vel_parallel)
             vel_parallel2 = tf_transformations.quaternion_multiply(vel_parallel, q)      
             vel_parallel = [vel_parallel2[0], vel_parallel2[1], vel_parallel2[2], vel_parallel2[3]]       

             
             vel_angle = numpy.arctan2(vel_parallel[0], vel_parallel[1])
             
             vel_parallel_speed = 2.0
             if(distToTarget < 6.0):
                  vel_parallel_speed = distToTarget/3.0
             if(vel_parallel_speed < 0.3):
                  vel_parallel_speed = 0.3
             
             vel_parallel_speed = vel_parallel_speed - vel_parallel_speed*minPoseDistance/3.0
             
             if(vel_parallel_speed < 0.3):
                  vel_parallel_speed = 0.3
             
             vel_parallel_mag = vel_parallel_speed/((vel_parallel[0])**2 + (vel_parallel[1])**2)**(1/2)
             
             vel_parallel[0] = vel_parallel[0]*vel_parallel_mag
             vel_parallel[1] = vel_parallel[1]*vel_parallel_mag
             
             if(numpy.isnan(vel_parallel[0])):
                  vel_parallel[0] = 0.0
             if(numpy.isnan(vel_parallel[1])):
                  vel_parallel[1] = 0.0
             
             self.get_logger().info('vel_parallel: ' + str(vel_parallel))              
             
             closestX = 0.0
             closestY = 0.0
             
             if(secondX != nextX and secondY != nextY):
                  m1 = (secondY - nextY)/(secondX - nextX)
                  m2 = -1/m1
                  closestX = (m2*selfX - m1*nextX + nextY - selfY)/(m2 - m1)
                  closestY = m2*(closestX - selfX) + selfY
             elif(secondX != nextX and secondY == nextY):
                  closestY = secondY
                  closestX = selfX
             elif(secondX == nextX and secondY != nextY):
                 closestX = secondX
                 closestY = selfY
             else:
                  closestX = nextX
                  closestY = nextY
                  
             

             vel_towards = [float(closestX - selfX), float(closestY - selfY), 0.0, 0.0]
             vel_towards = tf_transformations.quaternion_multiply(q_inv, vel_towards)
             vel_towards2 = tf_transformations.quaternion_multiply(vel_towards, q)      
             vel_towards = [vel_towards2[0], vel_towards2[1], vel_towards2[2], vel_towards2[3]]       
           
               
             speed_towards = minPoseDistance/2.0
             if(speed_towards > 2.0):
                 speed_towards = 2.0
             if(speed_towards < 0.05):
                 speed_towards = 0.05
                 
             speed_towards_angle = numpy.arctan2(vel_towards[0], vel_towards[1])
             
             #speed_towards = speed_towards - speed_towards
             if(speed_towards < 0.05):
                  speed_towards = 0.05
                            
             vel_towards_mag = speed_towards/((vel_towards[0])**2 + (vel_towards[1])**2)**(1/2)
                        
             vel_towards[0] = vel_towards[0]*vel_towards_mag
             vel_towards[1] = vel_towards[1]*vel_towards_mag      
             self.get_logger().info('vel_towards: ' + str(vel_towards))  
                               
             velToCommand = Twist()
             velToCommand.linear.x = vel_parallel[0] + vel_towards[0];
             velToCommand.linear.y = vel_parallel[1] + vel_towards[1];
             
             vel_angle = numpy.arctan2(velToCommand.linear.x, velToCommand.linear.y)
             
             velToCommand.linear.x = velToCommand.linear.x - velToCommand.linear.x*abs(vel_angle)/numpy.pi/2
             velToCommand.linear.y = velToCommand.linear.y - velToCommand.linear.y*abs(vel_angle)/numpy.pi/2
             '''
             speed = 1.5 - 1.5*abs(vel_angle)/numpy.pi
             
             if(distToTarget < 6.0):
                 speed = distToTarget/4.0
                 if(speed < 0.3):
                     speed = 0.3
             
             velToCommand_mag = speed/((velToCommand.linear.x)**2 + (velToCommand.linear.y)**2)**(1/2)
             
             velToCommand.linear.x = velToCommand.linear.x*velToCommand_mag
             velToCommand.linear.y = velToCommand.linear.y*velToCommand_mag
             '''
             self.cmd_vel_publisher.publish(velToCommand)
  
    def odometry_callback(self, msg):
        self.stateEstimate = msg       

        
def main(args=None):
    rclpy.init(args=args)

    cmd_vel_generator = cmdVelGenerator()

    rclpy.spin(cmd_vel_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_vel_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
