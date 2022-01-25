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
        self.get_logger().info('Distance to target: ' + str(numpy.sqrt(velocityX**2 + velocityY**2))) 
        velocityX = velocityX/1.5
        velocityY = velocityY/1.5
        if numpy.sqrt(velocityX**2 + velocityY**2) >= 4.0:
             velocityX = velocityX/numpy.sqrt(velocityX**2 + velocityY**2)*4.0
             velocityY = velocityY/numpy.sqrt(velocityX**2 + velocityY**2)*4.0
        targetVel = [velocityX, velocityY, 0.0, 0.0]
        
        q = [self.stateEstimate.pose.pose.orientation.x, self.stateEstimate.pose.pose.orientation.y, self.stateEstimate.pose.pose.orientation.z, self.stateEstimate.pose.pose.orientation.w]
        q_inv = q.copy()
        q_inv[0] = -q_inv[0]
        q_inv[1] = -q_inv[1]
        q_inv[2] = -q_inv[2]

        targetVel = tf_transformations.quaternion_multiply(q_inv, targetVel)
        targetVel = tf_transformations.quaternion_multiply(targetVel, q)
        
        targetForceY = (targetVel[1]/9.0 - currentVelY/3.0)
        targetForceX = (targetVel[0]/9.0 - currentVelX/3.0)

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

        self.get_logger().info('theta_targetHeading: ' + str(theta_targetHeading*180/numpy.pi))  
                        

        omega = [self.stateEstimate.twist.twist.angular.x, self.stateEstimate.twist.twist.angular.y, self.stateEstimate.twist.twist.angular.z, 0.0]  
        omega = tf_transformations.quaternion_multiply(q, omega)
        omega = tf_transformations.quaternion_multiply(omega, q_inv)        
        yawRate = omega[2]       
        
        targetTorque = (theta_targetHeading*0.8 - omega[2]*4.0)
        #self.get_logger().info('targetTorque: ' + str(targetTorque))  
                
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
        leftRearCmd.data = (-targetForceY + targetForceX + targetTorque)
        rightRearCmd.data = (targetForceY + targetForceX - targetTorque)
        
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
