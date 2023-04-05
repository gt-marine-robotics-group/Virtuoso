# Virtuoso Auxiliary

## Contents
- [Physical Auxiliary System](#physical-auxiliary-system)
- [Virtuoso Auxiliary Nodes](#virtuoso-auxiliary-nodes)
  - [ball_shooter_node.py](#ball\_shooter\_nodepy)
  - [water_shooter_node.py](#water\_shooter\_nodepy)
  - [aux_serial_node.py](#aux\_serial\_nodepy)
- [External Services](#external-services)
- [External Actions](#external-actions)
- [External Communication](#external-communication)
- [Parameters](#parameters)
  - [aux_serial.yaml](#aux\_serialyaml)
  - [ball_shooter.yaml](#ball\_shooteryaml)

## Physical Auxiliary System

The physical auxiliary system (currently only for roboboat) consists of a bilge pump and a ball shooter. The bilge pump is powered by a relay (which Virtuoso will indirectly turn on or off). The ball shooter contains two motors, each controlled with a separate ESC; again, Virtuoso indirectly controls the data sent to the ESCs and thus the speed of the motors.

## Virtuoso Auxiliary Nodes

### ball_shooter_node.py
This node, when a request is sent, will turn on the ball shooter to perform an action specified in the `ball_shooter.yaml` params. For example, shooting three balls with each motor running at certain speeds.

### water_shooter_node.py
This node, when a request is sent, will turn on the water shooter for a requested period of time.

### aux_serial_node.py
Because we cannot have both our motor controller Teensy and our auxiliary controller Arduino Due both running micro-ros at the same time, we must send our commands to the auxiliary controller through serial directly. This node subscribes to the outputs of the water and ball shooter nodes and sends a bit string to the Arduino Due controlling the auxiliary system.

The bit string contains 20 characters (20 bytes) and is encoded in the following format:
- Bytes 0-3: "AAAA"
- Bytes 4-7: Command for the ball shooter motor times 1000
- Bytes 8-11: Command for the ball sequencer motor times 1000
- Bytes 12-15: Command for the water shooter motor times 1000
- Bytes 16-19: "BBBB"

## External Services 

| Service | Service Type | Frame | Purpose |
|---------|--------------|-------|---------|
| shoot_water | [virtuoso_msgs/ShootWater](/virtuoso_msgs/srv/ShootWater.srv) | N/A | Activates the water shooter for a requested period of time (in seconds). |

## External Actions

| Action | Action Type | Frame | Purpose |
|---------|--------------|-------|---------|
| shoot_balls | [virtuoso_msgs/ShotBalls](/virtuoso_msgs/action/ShootBalls.action) | N/A | Shoots the balls in a manner specified in `ball_shooter.yaml`. |

## External Communication
There are no outputs from the auxiliary system that are subscribed to by other parts of Virtuoso or micro-ros. Instead, all outputs are sent to the Arduino Due directly via serial.

## Parameters

### aux_serial.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| auxiliary_aux_serial | serial_port | string | The port of the Arduino Due on the computer. |

### ball_shooter.yaml
| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| auxiliary_ball_shooter | sim | bool | Whether or not to use the simulation ball shooter. Simulation ball shooter only requires sending a command to shoot, while physical ball shooter requires controlling the speed of each motor. |
| auxiliary_ball_shooter | shooter_motor_topic | string | The topic for the ball shooter motor. |
| auxiliary_ball_shooter | loading_motor_topic | string | The topic for the loading (sequencer) motor. |
| auxiliary_ball_shooter | num_shots | int | The number of balls to shoot. |
| auxiliary_ball_shooter | between_load_time_gap | float | The number of seconds to pause the loading (sequencing) motor between shots. |
| auxiliary_ball_shooter | shooter_motor_speed | float | Speed to run the shooter motor at. Min is 0; max is 1.2. |
| auxiliary_ball_shooter | loading_motor_speed | float | Speed to run the loading motor at. Currently negative because loading motor needs to spin in reverse. Max absolute value is 1.0. |
| auxiliary_ball_shooter | shooter_motor_spinup_time | float | The number of seconds to wait for the shooter motor to spin up before shooting the first shot. | 
| auxiliary_ball_shooter | single_shot_time | float | The number of seconds to run the loading (sequencer) motor to fire one shot.