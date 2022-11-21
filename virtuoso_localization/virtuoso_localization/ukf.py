#from kalman_basic_interfaces.srv import State
#from kalman_basic_interfaces.srv import KalmanState

import numpy as np
import time as time
from random import seed
from random import gauss

#unscented kalman filter

def kalman_unscented(motionModelF, observationModel, xHatPrevious, U, dt, P0, W, V, z):
	#motionModelF should give X(t+dt) from inputs X(t), U(t), and dt. syntax: motionModelF(X, U,dt)
	#observationModel should give z from X. syntax: observationModel(X)
	#xHatPrevious is the previous time state estimate
	#U is the control input 
	#dt is the timestep 
	#P0 is the covariance matrix from the previous timestep
	#W is the motion model noise matrix
	#V is the sensor noise matrix
	#z is the current time sensor data 
	
	n = xHatPrevious.size
	#constants for the unscented transform
	alpha = 1*10**(-4)
	kap = 0
	lambda1 = (alpha**2)*(n+kap)-n
	gamma1 = (lambda1+n)**0.5
	beta=2
	
	W_m = np.zeros(2*n+1)
	W_c = np.zeros(2*n+1)
	#Set weights
	W_m.fill(1/(2*(n+lambda1)))
	W_c.fill(1/(2*(n+lambda1)))
	W_m[0] = lambda1/(n+lambda1)
	W_c[0] = lambda1/(n+lambda1) + (1-alpha**2+beta)
	#square root of the previous covariance matrix, must be positive definite
	sqrt_P = np.linalg.cholesky(P0)
	
	#find the sigma points around the current state
	Chi = np.zeros((n,2*n+1))
	Chi[0:,0] = xHatPrevious[0:,0]
	for i in range(0,n):

		Chi[0:,i+1] = xHatPrevious[0:,0].copy() + gamma1*sqrt_P[0:,i]
		Chi[0:,n+i+1] = xHatPrevious[0:,0].copy() - gamma1*sqrt_P[0:,i]

	
	#propagate sigma points forward in time using the motion model from t to t + dt
	Chi[0:,0] = motionModelF(Chi[0:,0],U,dt)[0:,0]
	for i in range(0,n):
		Chi[0:,i+1] = motionModelF(Chi[0:,i+1],U,dt)[0:,0]
		Chi[0:,n+i+1] = motionModelF(Chi[0:,n+i+1],U,dt)[0:,0]
	
	sum_x = np.zeros((n,1))
	#take a weighted average of the resulting propagated sigma points to get the updated state estimate according to the motion model
	for i in range(0,2*n+1):
		sum_x[0:,0] = sum_x[0:,0] + W_m[i]*Chi[0:,i]

	Pk = np.zeros((n,n))

	m = observationModel(Chi[0:,1].reshape((n,1))).size

	sum_y = np.zeros((m,1))
	Upsilon = np.zeros((m, 2*n+1))
	
	for i in range(0,2*n+1):
		#Calculate the process covariance matrix
		Pk = Pk + W_c[i]*np.matmul((Chi[0:,i] - sum_x[0:,0]).reshape(n,1),np.transpose((Chi[0:,i] - sum_x[0:,0]).reshape(n,1)))
		#find the observations corresponding to each propagated sigma point

		Upsilon[0:,i] = observationModel(Chi[0:,i].reshape((n,1)))[0:,0]

		#Find the weighted average of the observations to get the observation estimate
		sum_y[0:,0] = sum_y[0:,0] + W_m[i]*Upsilon[0:,i]

	Pk = Pk + W
	
	Pyy = np.zeros((m,m))
	Pxy = np.zeros((n,m))


	for i in range (0,2*n+1):
		#calculate the covariance of the observation sigma points
		#Pyy = Pyy + W_c[i]*np.matmul((Upsilon[0:,i] - sum_y[0:,0]).reshape(m,1),np.transpose((Upsilon[0:,i] - sum_y[0:,0]).reshape(m,1)))
		Pyy = Pyy + W_c[i]*np.matmul((Upsilon[0:,i] - Upsilon[0:,0]).reshape(m,1),np.transpose((Upsilon[0:,i] - Upsilon[0:,0]).reshape(m,1)))
		#The following is for if the observation is a simple scalar
		if ((Upsilon[0:,i] - sum_y).size) > 1:
			Pxy = Pxy + W_c[i]*np.matmul((Chi[0:,i] - sum_x[0:,0]).reshape(n,1),np.transpose((Upsilon[0:,i] - sum_y[0:,0]).reshape(m,1)))
		else:
			Pxy = Pxy + W_c[i]*(Upsilon[0:,i] - sum_y[0:,0])*(Chi[0:,i] - sum_x[0:,0])
	Pyy = Pyy + V #add on the sensor noise

	
	#calculate the kalman gain
	if Pyy.size>1:
		kappa = np.matmul(Pxy,np.linalg.inv(Pyy))
	else:
		kappa = Pxy/Pyy
	
	#calculate the updated state estimate
	if (z - sum_y).size>1:
		XHatnew = sum_x + np.matmul(kappa,(z - sum_y).reshape(m,1))

	else:
		XHatnew = sum_x + (kappa*((z - sum_y).reshape(m,1)))
	
	#Calculate the new covariance matrix
	if (z - sum_y).size>1:
		P = Pk - np.matmul(kappa,np.matmul(Pyy,np.transpose(kappa)))
	else:
		P = Pk - np.matmul(kappa,Pyy*np.transpose(kappa))

	
	return XHatnew, P

#All following code is for testing only
	
def basicMotion(X, U,dt):
	x_new = np.array([[0.0],[0.0],[0.0]])
	x_new[0] = X[0] + X[1]*dt + (1/2)*X[2]*dt**2
	x_new[1] = X[1] + X[2]*dt
	#accel = -0.01*x_new[1] + U
	#accel = 0.1
	x_new[2] = X[2]
	return x_new
	
def basicSensor(X):
	Xobserved = np.array([[0.0],[0.0]])
	Xobserved[0] = X[0]
	Xobserved[1] = X[2]
	return Xobserved
	
def basicMotion2(X, U,dt):
	x_new = np.array([[0.0],[0.0]])
	x_new[0] = X[0] + X[1]*dt
	x_new[1] = X[1] 
	return x_new
	
def basicSensor2(X):
	Xobserved = np.array([X[0]])
	Xobserved = Xobserved.reshape(-1, 1)
	return Xobserved
	
if __name__ == "__main__":
	X_i = np.array([[1.0],[2.0],[0.1]])

	Xhat_i = np.array([[1.0],[2.0],[0.1]])

	u = np.array([0.0])
	P0 = np.array([[50.0,10.0,10.0],[10.0,50.0,10.0],[10.0,10.0,50.0]])

	currentTime = 0.0

	seed(5)

	while True:
		
		X_i = basicMotion(X_i, 0, 0.01)

		print(X_i)
		currentTime = currentTime + 0.01
		W = np.array([[2.0,1.0,2.0],[1.0,2.0,1.0],[1.0,1.0,2.0]])
		V = np.array([[0.05, 0.05],[0.06, 0.05]])
		random1 = gauss(0,0.1) 
		random2 = gauss(0,0.01) 
		z = basicSensor(X_i)#+ np.array([[random1],[random2]])
		#print(z)
		#print(z)
		Xhat_i, P0 = kalman_unscented(basicMotion,basicSensor, Xhat_i, u,0.01,P0,W,V,z)
		print(Xhat_i)
		#print(P0)
		#print(currentTime)

		time.sleep(0.01)

