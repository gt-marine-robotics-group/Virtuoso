from matplotlib import pyplot as plt
#from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
import numpy as np
import pandas as pd
import os
import math


#Dynamic model that uses simplified 3-DOF model that uses Fossen's equations of motion for USVs
#Ref: R. Rameshbabu, M. Steffens and D. Mavris,
#     "System Identification Using the Modeling through Iterative Smoothing Algorithm,"
#     Global Oceans 2020: Singapore – U.S. Gulf Coast, 2020, pp. 1-7, doi: 10.1109/IEEECONF38699.2020.9389220.


class DynamicModel:

    def __init__(self):

        self.params=[5,5,0,20,0,0,0,1,0,0,20,0,0,20,]
        # X_u_dot = params[0]
        # Y_v_dot, Y_r_dot, Y_v, Y_v_v, Y_r, Y_r_r = (params[1], params[2], params[3], params[4], params[5], params[6])
        # N_r_dot, N_v_dot, N_v, N_r, N_r_r, N_v_v = (params[7], params[8], params[9], params[10], params[11], params[12])
        # X_u=params[13]

        self.mass=250.067 # kg
        self.width = 2.4  # meters
        self.length = 4.9  # meters
        self.Xcg = -0.009445  # meters
        self.Izz=499.99 # kg m^2

    #Set custom values for the parameters of the dynamic model

    def set_holonomic_model(self,params, mass, Xcg, Izz):
        self.params=params
        self.mass=mass
        self.Xcg=Xcg
        self.Izz=Izz




    # returns surge acceleration given surge velocity and surge control force
    def holonomic_model_speed(self,params, x, T, u0):
        Tx = T[:, 0]
        u, v, r = x[:, 0], x[:, 1], x[:, 2]
        xg=self.Xcg
        m=self.mass
        X_u_dot=params[0]
        Y_v_dot=params[1]
        X_u=params[13]
        N_v_dot=params[8]
        Y_r_dot=params[2]

        u_dot=1/(m-X_u_dot)*(Tx+X_u*(u-u0)+(m-Y_v_dot)*v*r+(m*xg+0.5*(N_v_dot+Y_r_dot))*r**2)

        return u_dot


    # returns sway acceleration and yaw acceleration given sway velocity,
    # yaw angle, sway control force input, and yaw moment input
    def holonomic_model_steering(self, params, x, T):
        Ty, Tn = T[:, 1], T[:, 2]
        u, v, r = x[:, 0], x[:, 1], x[:, 2]

        X_u_dot = params[0]
        Y_v_dot, Y_r_dot, Y_v, Y_v_v, Y_r, Y_r_r = (params[1], params[2],
                                                    params[3], params[4], params[5], params[6])
        N_r_dot, N_v_dot, N_v, N_r, N_r_r, N_v_v = (params[7], params[8],
                                                    params[9], params[10], params[11], params[12])

        xg=self.Xcg
        m=self.mass
        Izz=self.Izz

        v_dot=1/(m-Y_v_dot)*(Ty-(m-X_u_dot)*u*r-Y_v*v-Y_r*r-Y_v_v*math.fabs(v)*v-Y_r_r*math.fabs(r)*r)

        r_dot=1/(Izz-N_r_dot)*(-Tn+(m*xg-N_v_dot)*v_dot+m*xg*u*r-Y_v_dot*u*v-(0.5*(Y_r_dot+N_v_dot))*u*r +
                               X_u_dot*u*v+N_v*v+N_r*r+N_v_v*math.fabs(v)*v+N_r_r*math.fabs(r)*r)

        return v_dot, r_dot


    # run the dynamic model
    # inputs:
    # vel -> state vector consisting of surge velocity, sway velocity, and yaw-rate
    # T -> control force vector consisting of Surge Force, Sway Force, and Control Moment
    # u0 -> Initial surge velocity

    def run_holonomic_model(self, vel, T, u0):
        u_dot = self.holonomic_model_speed(self.params,vel, T, u0)
        v_dot, r_dot = self.holonomic_model_steering(self.params, vel, T)
        return u_dot, v_dot, r_dot