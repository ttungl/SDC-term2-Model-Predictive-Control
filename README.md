### SDC-term2
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
    
    Tung Thanh Le
    ttungl at gmail dot com
   
**Model Predictive Control Project**
---


#### Project description: 
* Choose the state, input(s), dynamics, constraints and implement Model Predictive Control. The outputs of the model are the steering angle and throttle acceleration which are applied to control the vehicle. So, the vehicle is able to drive successfully around the track.

* The implementation has been followed to the [rubric](https://review.udacity.com/#!/rubrics/896/view). 

* This is my result with [video demo](https://youtu.be/3PFlztq836s). I tested the model with velocity reference at 50 MPH and 70 MPH as shown below. The data output can be found at [here](https://github.com/ttungl/SDC-term2-Model-Predictive-Control/tree/master/datat_output).

<img src="https://github.com/ttungl/SDC-term2-Model-Predictive-Control/blob/master/gifs/gif_50.gif" height="149" width="270"> <img src="https://github.com/ttungl/SDC-term2-Model-Predictive-Control/blob/master/gifs/gif_70.gif" height="149" width="270"> 

---

#### MPC Model

This model is based on the kinetic model to simplify the situation. It doesn't include the forces of gravity effects, friction, or other external forces. 

* Update State variables:

		x[t+1] = (x[t] + v[t] * cos(psi[t]) * dt

		y[t+1] = (y[t] + v[t] * sin(psi[t]) * dt
		
		psi[t+1] = psi[t] - v[t] * delta[t] / Lf * dt
		
		v[t+1] = v[t] + a[t] * dt;
		
		cte[t+1] = (f[t] - y[t]) + v[t] * sin(epsi[t]) * dt;
		
		epsi[t+1] = (psi[t] - psides[t]) - v0[t]* delta[t] / Lf * dt;


* Timestep Length and Elapsed Duration (`N` & `dt`):
	+ Time step `N=10` and duration `dt=0.15`s are the perfect parameters for my implementation. If `N` and `dt` are either the larger or smaller amount of these settings, the vehicle doesn't keep running on the track, and it could be ended up to the lake, hill, or being stuck in a curb. So, these above settings are perfected to my cases.

* Number of constraints:
	+ I set `n_constraints = 6 * N`, where N is the timestep, and 6 element in a singular state vector.
	+ Lower bound and upper bound for variables, steering angle, and throttle acceleration are set at [`-1.0E19`,`1.0E19`], [`-deg2rad(25)`, `deg2rad(25)`], and [`-1.0`, `1.0`], respectively. 
	
* Polynomial Fitting and MPC Preprocessing:
	+ Transformating coordinates from global to local coordinates: This transforms coordinates from the map to the vehicle coordinates. (lines `105-110` in `main.cpp`) 
	+ Finding the third degree order polynomial and fitting to the waypoints. Using third degree order polynomial is generalized for most road scenarios. (line `116` in `main.cpp`)
	+ The outputs are fed to the `Solve()` method through a singular vector `state` to get the results, including `steering angle` and `throttle acceleration`. (line `133` in `main.cpp`)

* Latency: (for real-world scenarios) (lines `125-129` in `main.cpp`)
	+ `latency_value` = `0.1`. This latency value is `100 ms`. (line 125)
	+ `x_delay = v * cos(psi) * latency_value`.
    + `psi_delay = -v * steer_value / Lf * latency_value`.
    + `v_delay = v + throttle_value * latency_value`.






---