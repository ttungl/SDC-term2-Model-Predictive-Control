### SDC-term2
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
    
    Tung Thanh Le
    ttungl at gmail dot com
   
**Model Predictive Control Project**
---


#### Project description: 
* Choose the state, input(s), dynamics, constraints and implement Model Predictive Control. The vehicle is able to drive successfully around the track.

* The implementation has been followed to the [rubric](https://review.udacity.com/#!/rubrics/896/view). 

* This is my result with [video demo](https://youtu.be/3PFlztq836s). I tested the model with velocity reference at 50 MPH and 70 MPH as shown below. The data output can be found at [here](https://github.com/ttungl/SDC-term2-Model-Predictive-Control/tree/master/datat_output).

<img src="https://github.com/ttungl/SDC-term2-Model-Predictive-Control/blob/master/gifs/gif_50.gif" height="149" width="270"> <img src="https://github.com/ttungl/SDC-term2-Model-Predictive-Control/blob/master/gifs/gif_70.gif" height="149" width="270"> 

---

#### MPC Model implementation

* The model includes several components:
	
	+ This model is based on the kinetic model with 6 coefficients in a singular vector. The update equations are from lines `79-84` in `MPC.cpp`.
		* x: x-axis of the vehicle.
		* y: y-axis of the vehicle.
		* psi: the heading direction of the vehicle.
		* v: vehicle's velocity.
		* cte: cross track error.
		* epsi: orientation error.

	+ Constraints: `n_constraints = 6 * N`, where N is the timestep length. Lower bound and upper bound for variables, steering angle, and throttle acceleration are set at [`-1.0e19`,`1.0e19`], [`-deg2rad(25)`, `deg2rad(25)`], and [`-1.0`, `1.0`], respectively. 

	+ The optimization of the cost function is the sum of different terms, from lines `58-77` in `MPC.cpp`.
		
* Timestep Length and Elapsed Duration (`N` & `dt`):
	
	+ Timestep length `N` and elapsed duration `dt` are the parameters in the optimization section where the prediction horizon `T` = `N` x `dt`. 

		* A large `dt` can cause a continuous reference trajectory (discretization error).
	
		* A large `T` can be good for controlling but if predicting too far in the future, it's impractical to the realistic scenarios.
	
		* A large `T` and a small `dt` can lead to the large `N`, resulting in the higher cost of computation.
	
	+ In this project, after trying several times (try-and-error method), I observed that timestep length `N=10` and elapsed duration `dt=0.15`s are final parameters. 


* Polynomial Fitting and MPC Preprocessing:
	+ Transformating coordinates from global to local coordinates: This transforms coordinates from the map to the vehicle coordinates (lines `110-115` in `main.cpp`).
	
	+ Finding the third degree order polynomial and fitting to the waypoints. Using third degree order polynomial is generalized for most road scenarios. (lines ``47-66` and call it at line `116` in `main.cpp`)
	
	+ The outputs are fed to the `Solve()` method through a singular vector `state` to get the results, including `steering angle` and `throttle acceleration`. (line `138` in `main.cpp`)

* Latency: (for real-world scenarios) (lines `129-134` in `main.cpp`)
	+ `latency_value` = `0.1`. This latency value is `100 ms`. (line 127)
	
	+ `x_delay = v * cos(psi) * latency_value`.
    
    + `psi_delay = -v * steer_value / Lf * latency_value`.
    
    + `v_delay = v + throttle_value * latency_value`.

    + `pred_cte = cte + v * sin(epsi) * latency_value`.
    
    + `pred_epsi = epsi + v * delta * latency_value / Lf`.

---