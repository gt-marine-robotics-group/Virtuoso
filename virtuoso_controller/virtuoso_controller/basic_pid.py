import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import tf_transformations



#import pyproj
import numpy


class basicPID(Node):

    def __init__(self):
        super().__init__('basic_PID')
        
        self.stateEstimate = Odometry()
        self.targetWaypoint = Odometry()
        self.yawIntegral = 0.0
        self.xIntegral = 0.0
        self.yIntegral = 0.0
        self.previousTargetWaypoint = Odometry()
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
        
        self.targetForceXPub = self.create_publisher(Float32, '/controller/basic_pid/targetForceX', 10)
        self.targetForceYPub = self.create_publisher(Float32, '/controller/basic_pid/targetForceY', 10)
        self.targetTorquePub = self.create_publisher(Float32, '/controller/basic_pid/targetTorque', 10)
        
        #subscribe to odometry from localization
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/localization/odometry',
            self.odometry_callback,
            10)   
            
        #subscribe to waypoints
        self.waypoint_subscriber = self.create_subscription(
            Odometry,
            '/waypoint',
            self.waypoint_callback,
            10)     
        self.navigateToPoint_subscriber = self.create_subscription(
            Bool,
            '/navigation/navigateToPoint',
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
        self.targetWaypoint = msg        
        #if(self.receivedWaypoint == False):
             #self.timer = self.create_timer(0.1, self.run_pid())
        self.receivedWaypoint = True
        #self.run_pid()
        #self.get_logger().info('got waypoint') 

    def run_pid(self):
        targetX = self.targetWaypoint.pose.pose.position.x
        targetY = self.targetWaypoint.pose.pose.position.y    
        
        currentVelX = self.stateEstimate.twist.twist.linear.x
        currentVelY = self.stateEstimate.twist.twist.linear.y
    	
        currentX = self.stateEstimate.pose.pose.position.x
        currentY = self.stateEstimate.pose.pose.position.y
    	
        velocityX = targetX - currentX
        velocityY = targetY - currentY
        #self.get_logger().info('Distance to target: ' + str(numpy.sqrt(velocityX**2 + velocityY**2))) 

        targetVel = [velocityX, velocityY, 0.0, 0.0]
        
        q = [self.stateEstimate.pose.pose.orientation.x, self.stateEstimate.pose.pose.orientation.y, self.stateEstimate.pose.pose.orientation.z, self.stateEstimate.pose.pose.orientation.w]
        q_inv = q.copy()
        q_inv[0] = -q_inv[0]
        q_inv[1] = -q_inv[1]
        q_inv[2] = -q_inv[2]

        targetVel = tf_transformations.quaternion_multiply(q_inv, targetVel)
        targetVel = tf_transformations.quaternion_multiply(targetVel, q)
        
        #self.get_logger().info('targetx: ' + str(targetVel[0]))         
        #self.get_logger().info('targety: ' + str(targetVel[1])) 
                
        if(self.previousTargetWaypoint != self.targetWaypoint):
             self.xIntegral = 0.0
             self.yIntegral = 0.0
        self.xIntegral = self.xIntegral + targetVel[0]*0.01
        self.yIntegral = self.yIntegral + targetVel[1]*0.01       
        targetForceY = (targetVel[1]*0.15 - currentVelY*0.9) + self.yIntegral*0.000
        #self.get_logger().info('targetForceY: ' + str(targetForceY))  
        targetForceX = (targetVel[0]*0.11 - currentVelX*0.333) + self.xIntegral*0.000
	
        if(numpy.sqrt(velocityX**2 + velocityY**2) < 0.4):
             targetForceY = (targetVel[1]*0.15 - currentVelY*0.15) + self.yIntegral*0.000
             targetForceX = (targetVel[0]*0.11 - currentVelX*0.11) + self.xIntegral*0.000
        if(abs(targetForceY) < 0.2):
             targetForceY = targetForceY/abs(targetForceY)*0.2
        if(abs(targetForceX)<0.2):
             targetForceX = targetForceX/abs(targetForceX)*0.2
        theta_targetForce = numpy.arctan2(targetForceY, targetForceX)
        
        heading = [1.0, 0.0, 0.0, 0.0]
        q_target = [self.targetWaypoint.pose.pose.orientation.x, self.targetWaypoint.pose.pose.orientation.y, self.targetWaypoint.pose.pose.orientation.z, self.targetWaypoint.pose.pose.orientation.w]
        
        q_target_inv = q_target.copy()
        q_target_inv[0] = -q_target_inv[0]
        q_target_inv[1] = -q_target_inv[1]
        q_target_inv[2] = -q_target_inv[2]
        heading = tf_transformations.quaternion_multiply(q_target, heading)
        heading = tf_transformations.quaternion_multiply(heading, q_target_inv)
              
        heading = tf_transformations.quaternion_multiply(q_inv, heading)
        heading = tf_transformations.quaternion_multiply(heading, q)
        theta_targetHeading = numpy.arctan2(heading[1], heading[0])

        #self.get_logger().info('theta_targetHeading: ' + str(theta_targetHeading*180/numpy.pi))  
                        

        omega = [self.stateEstimate.twist.twist.angular.x, self.stateEstimate.twist.twist.angular.y, self.stateEstimate.twist.twist.angular.z, 0.0]  
        omega = tf_transformations.quaternion_multiply(q, omega)
        omega = tf_transformations.quaternion_multiply(omega, q_inv)        
        yawRate = omega[2]       
        
        if(self.previousTargetWaypoint != self.targetWaypoint):
             self.yawIntegral = 0.0
        self.yawIntegral = self.yawIntegral + theta_targetHeading*0.01
        #self.get_logger().info('yawIntegral: ' + str(self.yawIntegral))  
        
        targetTorque = (theta_targetHeading*0.76 - omega[2]*1.2 + 0.001*self.yawIntegral)
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
                                                            
        if(self.receivedWaypoint and self.navigateToPoint):
             self.get_logger().info('targetTorque: ' + str(targetTorque))   
             self.get_logger().info('Distance to target: ' + str(numpy.sqrt(velocityX**2 + velocityY**2))) 
             self.rightFrontPubAngle.publish(rightFrontAngle)
             self.leftRearPubAngle.publish(leftRearAngle)
             self.leftFrontPubAngle.publish(leftFrontAngle)
             self.rightRearPubAngle.publish(rightRearAngle)     
         
             self.leftFrontPubCmd.publish(leftFrontCmd)
             self.rightRearPubCmd.publish(rightRearCmd)      
             self.rightFrontPubCmd.publish(rightFrontCmd)
             self.leftRearPubCmd.publish(leftRearCmd)       
     	'''
        
        self.previousTargetWaypoint = self.targetWaypoint

        
def main(args=None):
    rclpy.init(args=args)

    basic_PID = basicPID()

    rclpy.spin(basic_PID)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    basic_PID.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
