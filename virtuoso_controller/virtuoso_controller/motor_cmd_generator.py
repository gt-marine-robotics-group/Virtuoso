from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import numpy

class MotorCmdGenerator:

    def __init__(self, sim_time, motor_config):

        self._sim_time = sim_time
        self.motor_config = motor_config

        self.state_estimate = Odometry()
        self.destination = Pose()
        self.navigate_to_point = False

        self.basic_force_x:float = None
        self.basic_force_y:float = None
        self.basic_torque:float = None
        self.vel_force_x:float = None
        self.vel_force_y:float = None
        self.vel_torque:float = None

        self._not_ready = True
    
    def run(self):
        if self._not_ready:
            if not (self.basic_force_x is None or self.basic_force_y is None 
                or self.basic_torque is None or self.vel_force_x is None
                or self.vel_force_y is None):
                self._not_ready = False
            #default to outputting 0 to the motors if a path has not been published
            else:
                return {
                    'right_front_angle': 0.0,
                    'right_rear_angle': 0.0,
                    'right_middle_angle': 0.0,
                    'left_front_angle': 0.0,
                    'left_rear_angle': 0.0,
                    'left_middle_angle': 0.0,
                    'right_front_cmd': 0.0,
                    'right_rear_cmd': 0.0,
                    'right_middle_cmd': 0.0,
                    'left_front_cmd': 0.0,
                    'left_rear_cmd': 0.0,
                    'left_middle_cmd': 0.0
                }

        left_front_angle = 0.0
        right_rear_angle = 0.0
        right_front_angle = 0.0
        left_rear_angle = 0.0
        left_middle_angle = 0.0
        right_middle_angle = 0.0

        left_front_cmd = 0.0
        right_rear_cmd = 0.0
        left_rear_cmd = 0.0
        right_front_cmd = 0.0
        left_middle_cmd = 0.0
        right_middle_cmd = 0.0
        
        #angle for simulation purposes only. Depends on wamv urdf
        left_front_angle = -90*numpy.pi/180*0
        right_rear_angle = 90*numpy.pi/180*0
        right_front_angle = 90*numpy.pi/180*0
        left_rear_angle = -90*numpy.pi/180*0

        
        #if navigate_to_point is true, use basic PID forcecommands
        #otherwise, use velocity pid force commands
        if (self.navigate_to_point):
                target_force_x = self.basic_force_x
                target_force_y = self.basic_force_y
        else:
                target_force_x = self.vel_force_x
                target_force_y = self.vel_force_y
        #always use basic PID torque commands
        target_torque = self.basic_torque if self.basic_torque is not None else 0
        
        #X drive configuration
        if (self.motor_config == "X"):        
             left_front_cmd = (-target_force_y + target_force_x - target_torque)
             right_front_cmd = (target_force_y + target_force_x + target_torque)
             #note the coefficient on target force y, which is to account for the difference in distance from the center of 
             #mass of the front and rear motors. Goal is that the torque from the front and rear motors cancel so that a target force y
             #results in pure translation
             left_rear_cmd = (target_force_y*0.6 + target_force_x - target_torque)
             right_rear_cmd = (-target_force_y*0.6 + target_force_x + target_torque)
             
        #six motor h drive configuration
        if (self.motor_config == "H"):        
             
            #if target force y has magnitude greater than 1.0, reduce the magnitude to 1.0
            if(abs(target_force_y) > 1.0):
                target_force_y = target_force_y/abs(target_force_y)
            
            #the angles are for simulation purposes only, just based on the wamv urdf
            left_front_angle = 0.785
            left_rear_angle = -0.785
            right_front_angle = -0.785
            right_rear_angle = 0.785
            
            # flipped!!!!! for roboboat
            left_front_cmd = (target_force_x + target_torque)
            right_front_cmd = (target_force_x - target_torque)
            left_rear_cmd = (target_force_x + target_torque)
            right_rear_cmd = (target_force_x - target_torque)
            left_middle_cmd = -target_force_y
            right_middle_cmd = target_force_y
            
        #We now want to normalize the commands so that the highest command has magnitude 1.0
        #This is for the firmware, which requests commands from -1 to 1
        highest_cmd = max(
            abs(left_front_cmd), 
            abs(right_front_cmd), 
            abs(left_rear_cmd), 
            abs(right_rear_cmd)
        )
        
        if(highest_cmd > 1.0):
            left_front_cmd /= highest_cmd
            right_front_cmd /= highest_cmd
            left_rear_cmd /= highest_cmd
            right_rear_cmd /= highest_cmd
            
        # Simulation expects values -100 to 100
        if self._sim_time:
            left_front_cmd *= 100
            right_front_cmd *= 100 
            left_rear_cmd *= 100 
            right_rear_cmd *= 100 
            left_middle_cmd *= 100
            right_middle_cmd *= 100

        return {
            'right_front_angle': right_front_angle,
            'right_rear_angle': right_rear_angle,
            'right_middle_angle': right_middle_angle,
            'left_front_angle': left_front_angle,
            'left_rear_angle': left_rear_angle,
            'left_middle_angle': left_middle_angle,
            'right_front_cmd': right_front_cmd,
            'right_rear_cmd': right_rear_cmd,
            'right_middle_cmd': right_middle_cmd,
            'left_front_cmd': left_front_cmd,
            'left_rear_cmd': left_rear_cmd,
            'left_middle_cmd': left_middle_cmd
        }
                                                            

# Not run normally, seems to be for debugging controller outside of ROS
if __name__ == "__main__":
     cmdgen = MotorCmdGenerator(True, "H")
     #cmdgen.vel_force_x = 0.0
     #cmdgen.vel_force_y = -2.0
     #cmdgen.basic_force_x = 0.0
     #cmdgen.basic_force_y = 0.0
     #cmdgen.basic_torque = 0.0  
     cmdgen.navigate_to_point = False   
     output = cmdgen.run()
     print(output['right_middle_cmd'])
