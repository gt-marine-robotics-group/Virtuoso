# State Estimation
Two common filters for state estimation, the **extended kalman filter** and **unscented kalman filter** are implemented. The **extended kalman filter** can also be used as a standard kalman filter. 

## Logic

### Kalman Filters
The function syntax is: 

```python
XHatnew, P = kalman_unscented(motionModelF, observationModel, xHatPrevious, U, dt, P0, W, V, z) 
```

```python
XHatnew, P = kalman_extended(motionModelF, observationModel, xHatPrevious, U, dt, P0, W, V, z)
```

The following inputs are unique to each timestep: 

&nbsp;&nbsp;&nbsp;&nbsp; **xHatPrevious** , **U**, **P0**, and **z**


The rest of the inputs will be the same at each timestep for the same system.

The functions output the estimated state and covariance matrix. 

### Variables
&nbsp;&nbsp;&nbsp;&nbsp; **xHatPrevious** = previous time state estimate 

&nbsp;&nbsp;&nbsp;&nbsp; **U** = control input 

&nbsp;&nbsp;&nbsp;&nbsp; **dt** = timestep 

&nbsp;&nbsp;&nbsp;&nbsp; **P0** = covariance matrix from the previous timestep

&nbsp;&nbsp;&nbsp;&nbsp; **W** = motion model noise matrix

&nbsp;&nbsp;&nbsp;&nbsp; **V** = sensor noise matrix

&nbsp;&nbsp;&nbsp;&nbsp; **z** = current time sensor data 

### Models

```motionModelF(X, U, dt)``` should return ```X(t+dt)``` from inputs ```X(t), U(t), and dt```. 

```observationModel(X)``` should return ```z``` from X. 




## Usage

The intended use of the filter functions is to find the updated estimated state and covariance matrix once per timestep, after a new observation is made using the sensors. The updated covariance matrix must be stored for use in the next update of the filter, as does the estimated state. 

