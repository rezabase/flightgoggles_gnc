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
  
Start's a small script to test a few trajectories: (NOTE: It doesnt follow them perfectly yet. I think its not tuned ok yet)
  roslaunch flightgoggles_gnc control_trajectory_test.launch
  

ToDO: 

- The UAV follows the trajectories but then it starts to bounce back and forth. I think the PIDvalues are not tuned yet. 

