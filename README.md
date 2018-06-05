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

This model had used the basic Car Kinematic model. Using Car current position, velocity, direction etc, its next status has been calculated. 

Number of points and the time interval define the prediction horizon. These parameters also impacts the output performance. With larger values will lead to sluggish output and may not give real driving feel.

Waypoints provided by the simulator are converted to the car coordinates. A polynomial is fitted to these transformed waypoints. Coefficients of this polynomial are used to calculate cross track error and Orientation error, which is used to create the reference trajectory.

To make steering smooth, we need to tune the steering value in the cost function. If the vehicle motion is not smooth and we find spikes in the steering angle value graph, we require to tune the cost function.

Later this model takes care for the latency, which is nothing but the time gap consideration between the desired output and actuation of the actuators. This make it closer to the real world driving behaviour. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.




## Output

![](https://github.com/ermadhukar/SDCND_T2_P5_Model_Predictive_Control/blob/master/Img/T2P5_MPC.png)

![](https://github.com/ermadhukar/SDCND_T2_P5_Model_Predictive_Control/blob/master/Img/T2P5_MPC2.png)

![](https://github.com/ermadhukar/SDCND_T2_P5_Model_Predictive_Control/blob/master/Img/T2P5_MPC4.png)


