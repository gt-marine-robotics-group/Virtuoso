from matplotlib import pyplot as plt
#from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
import numpy as np
import pandas as pd
import os
import math
from dynamic_model import DynamicModel

dynModelObj=DynamicModel()

params=[0,0,0,0,0,0,0,0,0,0,0,0,0]
mass=1
Xcg=0
Izz=1

dynModelObj.set_holonomic_model(params, mass, Xcg, Izz)

vel=[1 ,0, 0]
T=[0,1,0]
u0=0

u_dot, v_dot, r_dot=dynModelObj.run_holonomic_model(vel,T,u0)

