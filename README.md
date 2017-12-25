### SDC-term2
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
    
    Tung Thanh Le
    ttungl at gmail dot com
   
**Model Predictive Control Project**
---


#### Project description: 
* Choose the state, input(s), dynamics, constraints and implement Model Predictive Control. The outputs of the model are the steering angle and throttle acceleration which are applied to control the vehicle. So, the vehicle is able to drive successfully around the track.

* The implementation has been followed to the [rubric](https://review.udacity.com/#!/rubrics/896/view). 

---

#### MPC Model

This model is based on the kinetic model to simplify the situation. It doesn't include the forces of gravity effects, friction, or other external forces. 


##### **Update Equations**:

* State variables:



* Inputs


* Dynamics



##### **Constraints**:

* Time step and duration:
	+ Time step `N=10` and duration `dt=0.15` are the perfect parameters for my implementation. 
	+ 

* Polyfit:


* Polyeval:


* Latency:



---