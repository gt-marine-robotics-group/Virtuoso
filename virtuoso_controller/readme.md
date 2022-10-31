A controller package was created to follow paths from the navigation package. 

The controller consists of four main parts — an outer loop that commands a vehicle velocity, an inner loop that outputs vehicle force and torque commands, a last-meter PID controller that is intended for use once the vehicle is close to its target, and a control mixer that outputs vehicle thruster commands based on the force and torque commands. 

The target attitude is the direction such that the vehicle’s +x axis is parallel to the desired velocity until the vehicle comes within a specified distance of its target, at which point the goal attitude specified by navigation package is set as the target. 

### cmd_vel_generator

The outer loop takes the vehicle position and target path as its inputs. It calculates a desired velocity as a sum of two vectors. The first of these is a vector directly towards the line between the closest point on the path and the next point on the path. The second is the vector from the closest point on the path to the next point on the path. These vectors are scaled such that if the vehicle is further from the path, the vector towards the path is favored. In addition, the velocity magnitude is decreased if the vehicle is pointing away from the target velocity so that the vehicle can rotate itself to face in the direction of travel. This is preferred since the hydrodynamics of the vehicle favor motion in the surge direction. 

### velocity_pid

The main inner loop PID controller takes in the vehicle’s current velocity, target velocity, and target attitude and outputs a goal X and Y force as well as torque. 

### basic_pid

The last-meter PID controller takes in vehicle velocity, position, and attitude as well as target position and attitude and outputs X and Y force as well as torque commands. The last-meter PID controller is implemented so that station keeping can be tuned separately from the vehicle behavior between goals. 

### cmd_vel_generator

The control mixer is what sends individual thruster commands based on target X and Y force as well as torque. The rear thruster commands are scaled down in magnitude to not induce excessive torque with Y force commands. In addition, the commands to all thrusters are scaled so that the maximum magnitude is 1.0, but the ratios of all thruster commands are kept.
