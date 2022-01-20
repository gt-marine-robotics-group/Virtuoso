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

Xhat_i = np.zeros((9,1))
P0 = np.zeros((9,9))
P0.fill(0.1)
np.fill_diagonal(P0,0.5)


while True:
	
	W = np.zeros((9,9))
	W.fill(0.1)
	np.fill_diagonal(W,0.5)
	
	V = np.zeros((6,6))
	V.fill(0.1)
	np.fill_diagonal(V,0.5)
	
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
	#print(xHatPrevious)
	
	currentTime = currentTime + dt
	
	z = model.wamv_sensor_model(xHatPrevious.reshape((9,1))).reshape((6,1))
	
	Xhat_i, P0 = ukf.kalman_unscented(model.wamv_motion_model,model.wamv_sensor_model, Xhat_i, U,dt,P0,W,V,z)

	#print(Xhat_i)

	
	time.sleep(dt)

	#print(currentTime)
