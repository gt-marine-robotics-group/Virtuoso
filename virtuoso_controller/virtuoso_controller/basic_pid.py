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
        self.get_logger().info('theta_targetForce: ' + str(theta_targetForce*180/numpy.pi))   	
        #self.get_logger().info('targetForceMag: ' + str(targetForceY))
        targetForce = Float32()
        targetForce.data = targetForceMag/10.0
        #self.get_logger().info('targetForce: ' + str(targetForce))  
        msg1 = Float32()
        msg1.data = (45*numpy.pi/180+theta_targetForce)
        msg2 = Float32()
        msg2.data = -(45*numpy.pi/180-theta_targetForce)
        
        if theta_targetForce >=-135*numpy.pi/180 and theta_targetForce <=45*numpy.pi/180:
             self.leftFrontPubAngle.publish(msg1)
             self.rightRearPubAngle.publish(msg1)
             self.leftFrontPubCmd.publish(targetForce)
             self.rightRearPubCmd.publish(targetForce)
             #self.get_logger().info('Mode: Forward Right')  
        if theta_targetForce >=-45*numpy.pi/180 and theta_targetForce <=135*numpy.pi/180:
             self.rightFrontPubAngle.publish(msg2)
             self.leftRearPubAngle.publish(msg2)
             self.rightFrontPubCmd.publish(targetForce)
             self.leftRearPubCmd.publish(targetForce)
             #self.get_logger().info('Mode: Forward Left')  
        if theta_targetForce >-180*numpy.pi/180 and theta_targetForce <-135*numpy.pi/180:
             msg2.data = (90*numpy.pi/180)
             self.rightFrontPubAngle.publish(msg2)
             self.leftRearPubAngle.publish(msg2)
             self.rightFrontPubCmd.publish(targetForce)
             self.leftRearPubCmd.publish(targetForce)
             
             phi = theta_targetForce+180.0 + 45.0 
             phi = theta_targetForce+180.0 + phi              
             
             msg1.data = -(180 - 45 - phi)
             
             self.leftFrontPubAngle.publish(msg1)
             self.rightRearPubAngle.publish(msg1)
             self.leftFrontPubCmd.publish(targetForce)
             self.rightRearPubCmd.publish(targetForce)   
             self.get_logger().info('Mode: Back 1')  
        if theta_targetForce > 135*numpy.pi/180 and theta_targetForce < 180*numpy.pi/180:
             phi = -(theta_targetForce-180.0) + 45.0 
             phi = theta_targetForce-180.0 - phi     
             msg2.data = (180 - 45 - phi)
             self.rightFrontPubAngle.publish(msg2)
             self.leftRearPubAngle.publish(msg2)
             self.rightFrontPubCmd.publish(targetForce)
             self.leftRearPubCmd.publish(targetForce)
             
             phi = theta_targetForce+180.0 + 45.0 
             phi = theta_targetForce+180.0 + phi              
             
             msg1.data = -(90*numpy.pi/180)#(180 - 45 - phi)
             
             self.leftFrontPubAngle.publish(msg1)
             self.rightRearPubAngle.publish(msg1)
             self.leftFrontPubCmd.publish(targetForce)
             self.rightRearPubCmd.publish(targetForce)  
             self.get_logger().info('Mode: Back 2')  
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
