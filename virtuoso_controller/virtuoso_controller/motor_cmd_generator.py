import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import numpy

class motorCMDGenerator(Node):

#Generates the motor commands from the information from basic_PID, velocity_PID, and choose_PID

    def __init__(self):
        super().__init__('motor_cmd_generator')
        
        self.stateEstimate = Odometry()
        self.destination = Pose()
        self.navigateToPoint = False
        self.receivedCMD = False
        self.controllerRate = 0.1
        self.receivedBasic = False
        self.receivedVel = False
        
        self.navigateToPoint_subscriber = self.create_subscription(
            Bool,
            '/navigation/navigateToPoint',
            self.navigateToPoint_callback,
            10)           
        self.basicXSub = self.create_subscription(
            Float32,
            'controller/basic_pid/targetForceX',
            self.basicXcallback,
            10)      
        self.basicYSub = self.create_subscription(
            Float32,
            'controller/basic_pid/targetForceY',
            self.basicYcallback,
            10)     
        self.basicTorqueSub = self.create_subscription(
            Float32,
            'controller/basic_pid/targetTorque',
            self.basicTorquecallback,
            10)   
        self.velXSub = self.create_subscription(
            Float32,
            'controller/velocity_pid/targetForceX',
            self.velXcallback,
            10)  
        self.velYSub = self.create_subscription(
            Float32,
            'controller/velocity_pid/targetForceY',
            self.velYcallback,
            10)  
        self.velTorqueSub = self.create_subscription(
            Float32,
            'controller/velocity_pid/targetTorque',
            self.velTorquecallback,
            10)  

        self.leftFrontPubAngle = self.create_publisher(Float32, '/wamv/thrusters/left_front_thrust_angle', 10)
        self.rightFrontPubAngle = self.create_publisher(Float32, '/wamv/thrusters/right_front_thrust_angle', 10)
        self.leftRearPubAngle = self.create_publisher(Float32, '/wamv/thrusters/left_rear_thrust_angle', 10)
        self.rightRearPubAngle = self.create_publisher(Float32, '/wamv/thrusters/right_rear_thrust_angle', 10)

        self.leftFrontPubCmd = self.create_publisher(Float32, '/wamv/thrusters/left_front_thrust_cmd', 10)
        self.rightFrontPubCmd = self.create_publisher(Float32, '/wamv/thrusters/right_front_thrust_cmd', 10)             
        self.leftRearPubCmd = self.create_publisher(Float32, '/wamv/thrusters/left_rear_thrust_cmd', 10)
        self.rightRearPubCmd = self.create_publisher(Float32, '/wamv/thrusters/right_rear_thrust_cmd', 10)
        

        self.timer = self.create_timer(self.controllerRate, self.timer_callback)

    def navigateToPoint_callback(self, msg):
        self.navigateToPoint = msg.data

    def basicXcallback(self, msg):
        self.basicForceX = msg.data
        self.receivedBasic = True
        self.checkReceivedCmd()
    def basicYcallback(self, msg):
        self.basicForceY = msg.data
    def basicTorquecallback(self, msg):
        self.basicTorque = msg.data
    def velXcallback(self, msg):
        self.velForceX = msg.data
        self.receivedVel = True
        self.checkReceivedCmd()
    def velYcallback(self, msg):
        self.velForceY = msg.data              
    def velTorquecallback(self, msg):
        self.velTorque = msg.data

    def checkReceivedCmd(self):
        if(self.receivedBasic and self.receivedVel):
            self.receivedCMD = True
        
    def timer_callback(self):
        if(self.receivedCMD):
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

            if (self.navigateToPoint):
                targetForceX = self.basicForceX
                targetForceY = self.basicForceY
            else:
                targetForceX = self.velForceX
                targetForceY= self.velForceY
            targetTorque = self.basicTorque
            
                  
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

    motor_cmd_generator = motorCMDGenerator()

    rclpy.spin(motor_cmd_generator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_cmd_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
