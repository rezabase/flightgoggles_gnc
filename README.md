# flightgoggles_gnc

## Requirements: 

Must have the MIT flightgoggles package already installed. 
Also, in order to be able to reset the UAV position, I have modified the files in flightgoggles_uav_dynamics and added a reset topic to it. A copy of the modified files can be found in this repo, in a compressed file. 

## Launch Files:
Start using one of the following launch files: 

Only starts the control modules: 
  roslaunch flightgoggles_gnc control.launch

Starts the control module and Dynamic Reconfiguration for manual PID tuning:
  roslaunch flightgoggles_gnc control_manual_tune.launch
  
Start's a small script to test a few trajectories: (NOTE: It doesn’t follow them perfectly yet. I think its not tuned ok yet)
  roslaunch flightgoggles_gnc control_trajectory_test.launch
  
## Main nodes:

The main nodes in this package are 

### fg_conrol

This is the PID controller. 

### fg_estimation

I haven’t done much in this node yet, but prepared it to use it to filter the IMU and Laser altitude sensors and use them to estimate the UAVs location. We might need to find a way to use the images fromt he cameras as well to calculate the uav's current pose. 

## ToDO: 

- The UAV follows the trajectories but then it starts to bounce back and forth. I think the PID values are not tuned yet.
- Must not use the TF values for estimation of position and rotation. 
- and more... :-)

