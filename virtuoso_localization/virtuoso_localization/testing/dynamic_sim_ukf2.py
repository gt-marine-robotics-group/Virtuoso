import dynamic_model as dm
import numpy as np
import ukf
import ekf
import time
import random
import matplotlib as mpl
import matplotlib.pyplot as plt

model = dm.DynamicModel()

pos = np.array([0.0,0.0,0.0]).reshape(1,3)

dt = 0.01

vel = np.array([7.0,0.0,0.00]).reshape(1,3)
T = np.array([0,0,0]).reshape(1,3)
u0 = 0

currentTime = 0

Xhat_i = np.zeros((9,1))
Xhat_i[3] = 7.0
Xhat_i[4] = 0.0
Xhat_i[5] = 0.00
P0 = np.zeros((9,9))
P0.fill(0.1)
np.fill_diagonal(P0,0.5)

random.seed(5)

measTime = []
measState = []
actualState = []
filterState = []

while currentTime <= 10:
	
	measTime.append(currentTime)
	
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
	actualState.append(xHatPrevious)
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
	
	random1 = np.zeros((6,1))
	
	#random1[0:3,0] = np.random.normal(0, 0.1, (3,))
	#random1[2,0] = np.random.normal(0, 0.05)
	#random1[3:6,0] = np.random.normal(0, 0.05, (3,))
	#random1[5,0] = np.random.normal(0, 0.01)
	#z = z + random1
	
	measState.append(z)



	Xhat_i, P0 = ukf.kalman_unscented(model.wamv_motion_model,model.wamv_sensor_model, Xhat_i, U,dt,P0,W,V,z)
	filterState.append(Xhat_i)
	#print(Xhat_i)
	
	#time.sleep(dt)

	#print(currentTime)

measState = np.array(measState)
actualState = np.array(actualState)
filterState = np.array(filterState)

plot1 = plt.figure(1)
plt.plot(measTime, measState[:,3,:])
plt.plot(measTime, actualState[:,6,:])
plt.plot(measTime, filterState[:,6,:])
plt.title('Acceleration Surge')
plt.legend(['Measurement', 'Actual', 'Filtered'])


plot2 = plt.figure(2)
plt.plot(measTime, measState[:,0,:])
plt.plot(measTime, actualState[:,0,:])
plt.plot(measTime, filterState[:,0,:])
plt.title('Position Surge')
plt.legend(['Measurement','Actual', 'Filtered'])

plot3 = plt.figure(3)
plt.plot(measTime, actualState[:,3,:])
plt.plot(measTime, filterState[:,3,:])
plt.title('Velocity Surge')
plt.legend(['Actual', 'Filtered'])

plot1 = plt.figure(4)

plt.plot(measTime, actualState[:,8,:])
plt.plot(measTime, filterState[:,8,:])
plt.title('Acceleration Yaw')
plt.legend(['Actual', 'Filtered'])


plot2 = plt.figure(5)
plt.plot(measTime, measState[:,2,:])
plt.plot(measTime, actualState[:,2,:])
plt.plot(measTime, filterState[:,2,:])
plt.title('Position Yaw')
plt.legend(['Measurement','Actual', 'Filtered'])

plot3 = plt.figure(6)
plt.plot(measTime, measState[:,5,:])
plt.plot(measTime, actualState[:,5,:])
plt.plot(measTime, filterState[:,5,:])
plt.title('Velocity Yaw')
plt.legend(['Measured', 'Actual', 'Filtered'])

plt.show()
