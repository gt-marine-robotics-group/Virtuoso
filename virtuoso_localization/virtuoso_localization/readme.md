# State Estimation
Two common filters for state estimation, the extended kalman filter and unscented kalman filter are implemented. The extended kalman filter can also be used as a standard kalman filter. 

## Logic

### Kalman Filters
The function syntax is: 

```XHatnew, P = kalman_unscented(motionModelF, observationModel, xHatPrevious, U, dt, P0, W, V, z) ```

```XHatnew, P = kalman_extended(motionModelF, observationModel, xHatPrevious, U, dt, P0, W, V, z) ```

```
xHatPrevious is the previous time state estimate
U is the control input 
dt is the timestep 
P0 is the covariance matrix from the previous timestep
W is the motion model noise matrix
V is the sensor noise matrix
z is the current time sensor data 
```

### Models

```motionModelF(X, U,dt)``` should return ```X(t+dt)``` from inputs ```X(t), U(t), and dt```. 

```observationModel(X)``` should return ```z from X```. 


The following inputs are unique to each timestep: 
```
xHatPrevious
U
P0
z
```

The rest of the inputs will be the same at each timestep for the same system.

The functions output the estimated state and covariance matrix. 

## Usage

The intended use of the filter functions is to find the updated estimated state and covariance matrix once per timestep, after a new observation is made using the sensors. The updated covariance matrix must be stored for use in the next update of the filter, as does the estimated state. 

