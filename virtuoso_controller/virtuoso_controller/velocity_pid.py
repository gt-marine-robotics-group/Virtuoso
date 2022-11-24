import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import tf_transformations



#import pyproj
import numpy


class velocityPID(Node):

    def __init__(self):
        super().__init__('velocity_PID')

        self.declare_parameter('velocity_kp', 1.0)
        self.declare_parameter('velocity_kd', 1.0)
        self.declare_parameter('velocity_ki', 1.0)
        
        self.stateEstimate = Odometry()
        self.targetTwist = Twist()
        self.yawIntegral = 0.0
        self.xIntegral = 0.0
        self.yIntegral = 0.0
        self.previousTargetTwist = Twist()
        self.receivedWaypoint = False
        self.navigateToPoint = False
        '''
        self.leftFrontPubAngle = self.create_publisher(Float32, '/wamv/thrusters/left_front_thrust_angle', 10)
        self.rightFrontPubAngle = self.create_publisher(Float32, '/wamv/thrusters/right_front_thrust_angle', 10)
        self.leftRearPubAngle = self.create_publisher(Float32, '/wamv/thrusters/left_rear_thrust_angle', 10)
        self.rightRearPubAngle = self.create_publisher(Float32, '/wamv/thrusters/right_rear_thrust_angle', 10)

        self.leftFrontPubCmd = self.create_publisher(Float32, '/wamv/thrusters/left_front_thrust_cmd', 10)
        self.rightFrontPubCmd = self.create_publisher(Float32, '/wamv/thrusters/right_front_thrust_cmd', 10)             
        self.leftRearPubCmd = self.create_publisher(Float32, '/wamv/thrusters/left_rear_thrust_cmd', 10)
        self.rightRearPubCmd = self.create_publisher(Float32, '/wamv/thrusters/right_rear_thrust_cmd', 10)    
        '''
        self.targetForceXPub = self.create_publisher(Float32, '/controller/velocity_pid/targetForceX', 10)
        self.targetForceYPub = self.create_publisher(Float32, '/controller/velocity_pid/targetForceY', 10)
        self.targetTorquePub = self.create_publisher(Float32, '/controller/velocity_pid/targetTorque', 10)
        
        #subscribe to odometry from localization
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odometry_callback,
            10)   
            
        #subscribe to waypoints
        self.waypoint_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.waypoint_callback,
            10)    
        self.navigateToPoint_subscriber = self.create_subscription(
            Bool,
            '/controller/navigateToPoint',
            self.navigateToPoint_callback,
            10)      

        self.odom_subscriber
        self.waypoint_subscriber 
        self.timer2 = self.create_timer(0.01, self.run_pid)
        
        
    def navigateToPoint_callback(self, msg):
        self.navigateToPoint = msg.data
        	
    def odometry_callback(self, msg):
        self.stateEstimate = msg
    
    def waypoint_callback(self, msg):
        self.targetTwist = msg    
        #self.get_logger().info('targetTwist: ' + str((self.targetTwist)))    
        #if(self.receivedWaypoint == False):
             #self.timer = self.create_timer(0.1, self.run_pid())
        self.receivedWaypoint = True
        #self.run_pid()
        #self.get_logger().info('got waypoint') 

    def run_pid(self): 
        
        currentVelX = self.stateEstimate.twist.twist.linear.x
        currentVelY = self.stateEstimate.twist.twist.linear.y
    	
        currentX = self.stateEstimate.pose.pose.position.x
        currentY = self.stateEstimate.pose.pose.position.y
    	

        targetVel = [self.targetTwist.linear.x, self.targetTwist.linear.y , 0.0, 0.0]

                
        q = [self.stateEstimate.pose.pose.orientation.x, self.stateEstimate.pose.pose.orientation.y, self.stateEstimate.pose.pose.orientation.z, self.stateEstimate.pose.pose.orientation.w]
        q_inv = q.copy()
        q_inv[0] = -q_inv[0]
        q_inv[1] = -q_inv[1]
        q_inv[2] = -q_inv[2]

        #targetVel = tf_transformations.quaternion_multiply(q_inv, targetVel)
        #targetVel = tf_transformations.quaternion_multiply(targetVel, q)
        
        #self.get_logger().info('targetx: ' + str((targetVel[0])))       
        #self.get_logger().info('targety: ' + str((targetVel[1]))) 
                
        if(self.previousTargetTwist != self.targetTwist):
             self.xIntegral = 0.0
             self.yIntegral = 0.0
        self.xIntegral = self.xIntegral + (targetVel[0]- currentVelX)*0.01
        self.yIntegral = self.yIntegral + (targetVel[1] - currentVelY)*0.01  

        kp_factor = self.get_parameter('velocity_kp').value
        kd_factor = self.get_parameter('velocity_kd').value
        ki_factor = self.get_parameter('velocity_ki').value
             
        targetForceY = (targetVel[1]*1.5*kp_factor - currentVelY*kd_factor)*0.5 + self.yIntegral*0.01*ki_factor 
        targetForceX = (targetVel[0]*1.5*kp_factor - currentVelX*kd_factor)*0.5 + self.xIntegral*0.01*ki_factor
        #self.get_logger().info('targetForceX: ' + str(targetForceX))  
        #self.get_logger().info('targetForceY: ' + str(targetForceY))    
        
        theta_targetForce = numpy.arctan2(targetForceY, targetForceX)
                               

        omega = [self.stateEstimate.twist.twist.angular.x, self.stateEstimate.twist.twist.angular.y, self.stateEstimate.twist.twist.angular.z, 0.0]  
        omega = tf_transformations.quaternion_multiply(q, omega)
        omega = tf_transformations.quaternion_multiply(omega, q_inv)        
        yawRate = omega[2]       
        
        targetYawRate = self.targetTwist.angular.z
        
        if(self.previousTargetTwist != self.targetTwist):
             self.yawIntegral = 0.0
        self.yawIntegral = self.yawIntegral + (targetYawRate - yawRate)*0.01
        #self.get_logger().info('yawIntegral: ' + str(self.yawIntegral))  
        #self.get_logger().info('yawVelDiff: ' + str((yawRate - targetYawRate)))
        #self.get_logger().info('targetYawRate: ' + str((targetYawRate)))
        targetTorque = (targetYawRate - yawRate)*3.0 + 0.01*self.yawIntegral
        #self.get_logger().info('targetTorque: ' + str(targetTorque))  
        
        targetXToSend = Float32()
        targetYToSend = Float32()
        targetTorqueToSend = Float32()
        
        targetXToSend.data = targetForceX
        targetYToSend.data = targetForceY
        targetTorqueToSend.data = targetTorque
        if(self.receivedWaypoint):
             self.targetForceXPub.publish(targetXToSend)
             self.targetForceYPub.publish(targetYToSend)
             self.targetTorquePub.publish(targetTorqueToSend)
        
        '''
                
        leftFrontAngle = Float32()
        rightRearAngle = Float32()
        rightFrontAngle = Float32()      
        leftRearAngle = Float32()
        
        leftFrontCmd = Float32()
        rightRearCmd = Float32()
        leftRearCmd = Float32()
        rightFrontCmd = Float32()

        leftFrontAngle.data = -90*numpy.pi/180
        rightRearAngle.data = 90*numpy.pi/180
        rightFrontAngle.data = 90*numpy.pi/180
        leftRearAngle.data = -90*numpy.pi/180
        
        
                     
        leftFrontCmd.data = (-targetForceY - targetForceX - targetTorque)

        rightFrontCmd.data = (targetForceY - targetForceX + targetTorque)
        leftRearCmd.data = (-targetForceY*0.9 + targetForceX + targetTorque)
        rightRearCmd.data = (targetForceY*0.9 + targetForceX - targetTorque)
        
        if(leftFrontCmd.data <0):
             leftFrontCmd.data = leftFrontCmd.data*2.5
        if(rightFrontCmd.data <0):
             rightFrontCmd.data = rightFrontCmd.data*2.5
        if(leftRearCmd.data <0):
             leftRearCmd.data = leftRearCmd.data*2.5
        if(rightRearCmd.data <0):
             rightRearCmd.data = rightRearCmd.data*2.5
                                                            
        if(self.receivedWaypoint and not(self.navigateToPoint)):
             self.get_logger().info('targetTwist: ' + str((self.targetTwist)))   
             self.rightFrontPubAngle.publish(rightFrontAngle)
             self.leftRearPubAngle.publish(leftRearAngle)
             self.leftFrontPubAngle.publish(leftFrontAngle)
             self.rightRearPubAngle.publish(rightRearAngle)     
         
             self.leftFrontPubCmd.publish(leftFrontCmd)
             self.rightRearPubCmd.publish(rightRearCmd)      
             self.rightFrontPubCmd.publish(rightFrontCmd)
             self.leftRearPubCmd.publish(leftRearCmd)       
        '''
        self.previousTargetTwist = self.targetTwist

        
def main(args=None):
    rclpy.init(args=args)

    velocity_PID = velocityPID()

    rclpy.spin(velocity_PID)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_PID.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
