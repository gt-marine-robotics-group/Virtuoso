import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf_transformations



#import pyproj
import numpy


class basicPID(Node):

    def __init__(self):
        super().__init__('basic_PID')
        
        self.stateEstimate = Odometry()
        self.targetWaypoint = Odometry()

        self.leftFrontPubAngle = self.create_publisher(Float32, '/wamv/thrusters/left_front_thrust_angle', 10)
        self.rightFrontPubAngle = self.create_publisher(Float32, '/wamv/thrusters/right_front_thrust_angle', 10)
        self.leftRearPubAngle = self.create_publisher(Float32, '/wamv/thrusters/left_rear_thrust_angle', 10)
        self.rightRearPubAngle = self.create_publisher(Float32, '/wamv/thrusters/right_rear_thrust_angle', 10)

        self.leftFrontPubCmd = self.create_publisher(Float32, '/wamv/thrusters/left_front_thrust_cmd', 10)
        self.rightFrontPubCmd = self.create_publisher(Float32, '/wamv/thrusters/right_front_thrust_cmd', 10)             
        self.leftRearPubCmd = self.create_publisher(Float32, '/wamv/thrusters/left_rear_thrust_cmd', 10)
        self.rightRearPubCmd = self.create_publisher(Float32, '/wamv/thrusters/right_rear_thrust_cmd', 10)    
        
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
        self.odom_subscriber
        self.waypoint_subscriber
        


    def odometry_callback(self, msg):
        self.stateEstimate = msg
    
    def waypoint_callback(self, msg):
        self.targetWaypoint = msg        
        self.run_pid()

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
        if numpy.sqrt(velocityX**2 + velocityY**2) >= 4.0:
             velocityX = velocityX/numpy.sqrt(velocityX**2 + velocityY**2)*4.0
             velocityY = velocityY/numpy.sqrt(velocityX**2 + velocityY**2)*4.0
        targetVel = [velocityX, velocityY, 0.0, 0.0]
        #self.get_logger().info('targetVel: ' + str(targetVel)) 
        q = [self.stateEstimate.pose.pose.orientation.x, self.stateEstimate.pose.pose.orientation.y, self.stateEstimate.pose.pose.orientation.z, self.stateEstimate.pose.pose.orientation.w]

    	
        q_inv = q.copy()
        q_inv[0] = -q_inv[0]
        q_inv[1] = -q_inv[1]
        q_inv[2] = -q_inv[2]

        targetVel = tf_transformations.quaternion_multiply(q_inv, targetVel)
        targetVel = tf_transformations.quaternion_multiply(targetVel, q)
        
        targetForceY = targetVel[1] - currentVelY
        targetForceX = targetVel[0] - currentVelX
        targetForceMag = numpy.sqrt(targetForceY**2 + targetForceX**2)
        #self.get_logger().info('targetVel: ' + str(targetVel))   	
        theta_targetForce = numpy.arctan2(targetForceY, targetForceX)

        #self.get_logger().info('vehicle: ' + str(theta_vehicle))
        #self.get_logger().info('theta_targetForce: ' + str(theta_targetForce*180/numpy.pi))   	
        #self.get_logger().info('targetForceMag: ' + str(targetForceY))
        targetForce = Float32()
        targetForce.data = targetForceMag/5.0
        #self.get_logger().info('targetForce: ' + str(targetForce))  
        
        heading = [1.0, 0.0, 0.0, 0.0]
        q_target = [self.targetWaypoint.pose.pose.orientation.x, self.targetWaypoint.pose.pose.orientation.y, self.targetWaypoint.pose.pose.orientation.z, self.targetWaypoint.pose.pose.orientation.w]
        
        q_target_inv = q_target.copy()
        q_target_inv[0] = -q_target_inv[0]
        q_target_inv[1] = -q_target_inv[1]
        q_target_inv[2] = -q_target_inv[2]
        heading = tf_transformations.quaternion_multiply(q_target, heading)
        heading = tf_transformations.quaternion_multiply(heading, q_target_inv)
        
        self.get_logger().info('q_target_inv: ' + str(q_target_inv)) 
        self.get_logger().info('q_target: ' + str(q_target)) 
        self.get_logger().info('heading: ' + str(heading))          
        heading = tf_transformations.quaternion_multiply(q_inv, heading)
        #self.get_logger().info('heading: ' + str(heading))  
        heading = tf_transformations.quaternion_multiply(heading, q)

        theta_targetHeading = numpy.arctan2(heading[1], heading[0])
        #self.get_logger().info('heading: ' + str(heading))  
        self.get_logger().info('theta_targetHeading: ' + str(theta_targetHeading*180/numpy.pi))  
                        

        omega = [self.stateEstimate.twist.twist.angular.x, self.stateEstimate.twist.twist.angular.y, self.stateEstimate.twist.twist.angular.z, 0.0]  
        
        omega = tf_transformations.quaternion_multiply(q, omega)
        #self.get_logger().info('heading: ' + str(heading))  
        omega = tf_transformations.quaternion_multiply(omega, q_inv)        
        
        yawRate = omega[2]
        
        #self.get_logger().info('yawRate: ' + str(yawRate))  
        
        targetTorque = (theta_targetHeading - omega[2]*4.0)*0.5
        self.get_logger().info('targetTorque: ' + str(targetTorque))  
                
        leftFrontAngle = Float32()
        rightRearAngle = Float32()
        rightFrontAngle = Float32()      
        leftRearAngle = Float32()
        
        leftFrontCmd = Float32()
        rightRearCmd = Float32()
        leftRearCmd = Float32()
        rightFrontCmd = Float32()
        
        if theta_targetForce >=-135*numpy.pi/180 and theta_targetForce <=45*numpy.pi/180:
             leftFrontAngle.data = (45*numpy.pi/180+theta_targetForce)
             rightRearAngle.data = (45*numpy.pi/180+theta_targetForce)
             leftFrontCmd.data = targetForce.data.copy()
             rightRearCmd.data = targetForce.data.copy()
             #self.get_logger().info('Mode: Forward Right')  
        if theta_targetForce >=-45*numpy.pi/180 and theta_targetForce <=135*numpy.pi/180:
             rightFrontAngle.data = -(45*numpy.pi/180-theta_targetForce)
             leftRearAngle.data = -(45*numpy.pi/180-theta_targetForce)
             rightFrontCmd.data = targetForce.data.copy()
             leftRearCmd.data = targetForce.data.copy()
             #self.get_logger().info('Mode: Forward Left')  
        if theta_targetForce >-180*numpy.pi/180 and theta_targetForce <-135*numpy.pi/180:
             rightFrontAngle.data = (90*numpy.pi/180)
             leftRearAngle.data = (90*numpy.pi/180)
             rightFrontCmd.data = targetForce.data.copy()
             leftRearCmd.data = targetForce.data.copy()
             
             phi = theta_targetForce+180.0 + 45.0 
             phi = theta_targetForce+180.0 + phi              
             
             leftFrontAngle.data = -(180 - 45 - phi)
             rightRearAngle.data = -(180 - 45 - phi)
             leftFrontCmd.data = targetForce.data.copy()
             rightRearCmd.data = targetForce.data.copy()
             #self.get_logger().info('Mode: Back 1')  
        if theta_targetForce > 135*numpy.pi/180 and theta_targetForce < 180*numpy.pi/180:
             phi = -(theta_targetForce-180.0) + 45.0 
             phi = theta_targetForce-180.0 - phi     
             rightFrontAngle.data = (180 - 45 - phi)
             leftRearAngle.data = (180 - 45 - phi)

             rightFrontCmd.data = targetForce.data.copy()
             leftRearCmd.data = targetForce.data.copy()
             
             phi = theta_targetForce+180.0 + 45.0 
             phi = theta_targetForce+180.0 + phi              
             
             leftFrontAngle.data = -(90*numpy.pi/180)#(180 - 45 - phi)
             rightRearAngle.data = -(90*numpy.pi/180)#(180 - 45 - phi)

             leftFrontCmd.data = targetForce.data.copy()
             rightRearCmd.data = targetForce.data.copy()  
             #self.get_logger().info('Mode: Back 2')  
        if numpy.abs(targetTorque) > 1:
             targetTorque = targetTorque/numpy.abs(targetTorque)     
        if targetTorque < 0:
             leftFrontAngle.data = leftFrontAngle.data - (45*numpy.pi/180 + leftFrontAngle.data)*(-targetTorque)/4
             leftRearAngle.data = leftRearAngle.data + (45*numpy.pi/180 - leftRearAngle.data)*(-targetTorque)/4
             leftFrontCmd.data = leftFrontCmd.data - targetTorque/2
             leftRearCmd.data = leftRearCmd.data - targetTorque/2
        if targetTorque > 0:
             rightFrontAngle.data = rightFrontAngle.data + (45*numpy.pi/180 - rightFrontAngle.data)*(targetTorque)/4
             rightRearAngle.data = rightRearAngle.data - (45*numpy.pi/180 + rightRearAngle.data)*(targetTorque)/4    
             rightFrontCmd.data = rightFrontCmd.data + targetTorque/2
             rightRearCmd.data = rightRearCmd.data + targetTorque/2
        self.rightFrontPubAngle.publish(rightFrontAngle)
        self.leftRearPubAngle.publish(leftRearAngle)
        self.leftFrontPubAngle.publish(leftFrontAngle)
        self.rightRearPubAngle.publish(rightRearAngle)     
         
        self.leftFrontPubCmd.publish(leftFrontCmd)
        self.rightRearPubCmd.publish(rightRearCmd)      
        self.rightFrontPubCmd.publish(rightFrontCmd)
        self.leftRearPubCmd.publish(leftRearCmd)       
                                     
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
