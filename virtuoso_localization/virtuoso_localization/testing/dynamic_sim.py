import dynamic_model as dm
import numpy as np
import ukf
import ekf
import time

model = dm.DynamicModel()

pos = np.array([0.0,0.0,0.0]).reshape(1,3)

dt = 0.01

vel = np.array([5.0,0.5,0.01]).reshape(1,3)
T = np.array([0,0,0]).reshape(1,3)
u0 = 0

currentTime = 0

while True:

	u_dot, v_dot, r_dot = model.run_holonomic_model(vel, T, u0)
	
	U = T.reshape((3,1))
	xHatPrevious = np.array([[pos[0,0]],[pos[0,1]],[pos[0,2]],[vel[0,0]],[vel[0,1]],[vel[0,2]],[0],[0],[0]])
	xHatPrevious[6] = u_dot
	xHatPrevious[7] = v_dot
	xHatPrevious[8] = r_dot
	motionModelResult = model.wamv_motion_model(xHatPrevious,U,dt)
	
	pos[0,0] = pos[0,0] + vel[0,0]*dt + 0.5*u_dot*dt**(2)
	pos[0,1] = pos[0,1] + vel[0,1]*dt + 0.5*v_dot*dt**(2)
	pos[0,2] = pos[0,2] + vel[0,2]*dt + 0.5*r_dot*dt**(2)

	vel[0,0] = vel[0,0] + u_dot*dt
	vel[0,1] = vel[0,1] + v_dot*dt
	vel[0,2] = vel[0,2] + r_dot*dt
	u0 = 0
	
	currentTime = currentTime + dt

	
	time.sleep(dt)
	print(vel)
	#print(currentTime)
