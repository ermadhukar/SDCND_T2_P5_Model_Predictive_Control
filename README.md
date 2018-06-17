# UDACITY SDCND: Model Predictive Control
---
The objective of this project was to use Model Predictive Control Method to drive the car in autonomous mode inside the simulator. The code has been implemented in C++. The model has provided the latency to the car actuator to make it similar to real world behaviour.


## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* uWebSockets
* Ipopt and CppAD
* Simulator

## The Model
This model had used the basic Car Kinematic model. Using Car current position, velocity, direction etc, its next status has been calculated. This model doesn’t consider the the detailed tire-road dynamics. 
Following are the model equations:
```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt  
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt  
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt  
v[t] = v[t-1] + a[t-1] * dt  
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt  
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt  
```

Here, model states are: 

x: Position  
y: Position   
psi: Orientation  
v: Velocity  
cte: Cross-track error  
epsi: Orientation error   

Other parameters is, Lf: Distance between Car CG and front axle and, dt:  Time duration for the prediction.

Following are control input:  
a : Acceleration  
delta : Steering angle   

This model uses simplified vehicle dynamics model and does not account for complex  cornering forces which causes tire slip.


## Timestep Length (N) and Elapsed Duration (dt)

The prediction horizon ‘T’ is the duration over which future predictions are made. T is the product of two other variables, timesteps ‘N’ and elapsed time duration ‘dt’.  
For example, if N were 20 and dt were 0.5, then T would be 10 seconds.  
N, dt, and T are the hyperparameters which we need to tune for each model predictive controller. There are some general guidelines regarding these hyperparameters value selection are given. T should be as large as possible, while dt should be as small as possible. With proper value, model will track the path for both low and high speed closely.   
Different values have been tried in this MPC model. With higher values of N (e.g. 20), dt(e.g. 0.5), car started oscilating and for lower values of N(e.g. 5), dt (e.g. 0.5), car could not turn closely to predicted path.

Eventually the value of N(10) and dt(0.1) prove to be a good hyperparameters value, which has given a decent MPC model output.


## Polynomial Fitting and MPC Preprocessing

The provided waypoints are transformed into vehicle space and then fitted to a polynomial Using the polyfit() function.  A 3rd degree polynomial line is fit to these transformed waypoints. The polynomial coefficients are identified using the polyeval function to calculate the cte and epsi. These are used to create the reference trajectory for the car to travel. 


## Model Predictive Control with Latency

In a real car, an actuation command doesn’t execute instantly. There will be some delay in the actuation as the command propagates through the system. This situation is called "latency".
To make this model more accurate, there is a latency model integrated in this MPC model, which takes care of this latency. It involves predicting the state of vehicle after latency and actuating the actuators at this new corrected predicted state.
A realistic delay is in the order of 100 milliseconds (0.1 sec), which has been used as latency in the model.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.




## Output

![](https://github.com/ermadhukar/SDCND_T2_P5_Model_Predictive_Control/blob/master/Img/T2P5_MPC.png)

![](https://github.com/ermadhukar/SDCND_T2_P5_Model_Predictive_Control/blob/master/Img/T2P5_MPC2.png)

![](https://github.com/ermadhukar/SDCND_T2_P5_Model_Predictive_Control/blob/master/Img/T2P5_MPC4.png)


