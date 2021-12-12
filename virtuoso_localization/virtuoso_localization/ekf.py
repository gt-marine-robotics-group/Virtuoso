#from kalman_basic_interfaces.srv import State
#from kalman_basic_interfaces.srv import KalmanState

import numpy as np
import time as time
from random import seed
from random import gauss

def linearizedDynamics(f, X, U, dt):
	#This function finds the matrices A and B in the equation X(t + dt) = AX(t) + BU(t). A = Phi
	#X is the current state, U is the current control matrix, dt is the timestep
	#f is the motion model 
	n = X.size
	m = U.size
	dx = 0.01
	du = 0.01
	
	F_x = np.zeros((n,n))
	F_u = np.zeros((m,n))
	
	#Find the A matrix
	
	for i in range(0,n):
		delta_x = np.zeros((n,1))
		delta_x[i,0] = dx
		F_x[i] = np.transpose((f(X + delta_x, U,dt) - f(X - delta_x, U,dt))) / (2*dx)
	
	Phi = np.transpose(F_x)
	
	#Find the B matrix
	
	for i in range(0,m):
		delta_u = np.zeros((m,1))
		delta_u[i,0] = du
		F_u[i] = np.transpose((f(X, U + delta_u,dt) - f(X, U - delta_u,dt))) / (2*du)
	
	B = np.transpose(F_u)
	return Phi, B

def linearizedSensor(s, X):
	#This function finds the matrix C in z = CX, where z is the ideal sensor data output and X is the true state
	n = X.size
	
	dx = 0.01
	
	S_x = np.zeros((n,n))

	#Find the C Matrix
	for i in range(0,n):
		delta_x = np.zeros((n,1))
		delta_x[i,0] = dx
		S_x[i] = np.transpose((s(X + delta_x) - s(X - delta_x))) / (2*dx)
		
	C = np.transpose(S_x)
	
	return C

def kalman_extended(motionModelF, observationModel, xHatPrevious, U, dt, P0, W, V, z):
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
	#Find the linearized motion model, X(t + dt) = AX(t) + BU(t)
	A, B = linearizedDynamics(motionModelF, xHatPrevious, U,dt)
	#Find the linearized sensor model, z(t) = CX(t)
	C = linearizedSensor(observationModel, xHatPrevious)
	
	#Find X(t+dt) from X(t) and U(t). And dt.
	XHatnew = motionModelF(xHatPrevious,U,dt)
	#Predicted covariance matrix
	P = np.matmul(np.matmul(A,P0),np.transpose(A)) + W
	#Sensor covariance matrix 
	S = np.matmul(np.matmul(C,P),np.transpose(C)) + V
	#Kalman gain calculation 
	K_f = np.matmul(np.matmul(P,np.transpose(C)),np.linalg.inv(S))
	residual = z - np.matmul(C,XHatnew)
	#new state estimate
	XHatnew = XHatnew + np.matmul(K_f, residual)
	#new covariance matrix
	P = np.matmul((np.eye(n) - np.matmul(K_f,C)),P)
	return XHatnew, P
		
		
#The following functions are for testing only
	
def basicMotion(X, U,dt):
	x_new = np.array([[0.0],[0.0],[0.0]])
	x_new[0] = X[0] + X[1]*dt + (1/2)*X[2]*dt**2
	x_new[1] = X[1] + X[2]*dt
	#accel = -0.01*x_new[1] + U
	accel = 0
	x_new[2] = accel
	return x_new
	
def basicSensor(X):
	Xobserved = np.array([[0.0],[0.0],[0.0]])
	Xobserved[0] = X[0]
	Xobserved[1] = 0
	Xobserved[2] = X[2]
	return Xobserved
	
def basicMotion2(X, U,dt):
	x_new = np.array([[0.0],[0.0]])
	x_new[0] = X[0] + X[1]*dt
	x_new[1] = X[1] 
	return x_new
	
def basicSensor2(X):
	Xobserved = np.array([[0.0],[0.0]])
	Xobserved[0] = X[0]
	Xobserved[1] = 0
	return Xobserved

if __name__ == "__main__": #Test particle 

	X_i = np.array([[1.0],[2.0]])
	Xhat_i = np.array([[0.0],[0.0]])
	u = np.array([0.0])
	linearizedDynamics(basicMotion2,X_i,u,0.01)
	linearizedSensor(basicSensor2,X_i)
	P0 = np.array([[10.0,0.0],[0.0,10.0]])

	currentTime = 0.0

	seed(5)

	while True:
		
		X_i = basicMotion2(X_i, 0, 0.01)
		currentTime = currentTime + 0.01
		W = np.array([[1.0,0.0],[0.0,1.0]])
		V = np.array([[3.0],[10000000000000.0]])
		
		random1 = gauss(0,0.5) 
		
		z = basicSensor2(X_i + np.array([[random1],[0]]))
		#print(z)
		Xhat_i, P0 = kalman_extended(basicMotion2,basicSensor2, Xhat_i, u,0.01,P0,W,V,z)
		print(Xhat_i)
		print(currentTime)
		time.sleep(0.01)
