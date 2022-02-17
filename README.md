The 'paper' branch of this repo contains code accompanying the following paper:

"Online Kinematic Calibration for Legged Robots",

by Shuo Yang. We proposed a method to let legged robots do online calibration of certain kinematic parameters such as leg length that are hard to measure due to foot deformation. The repo provide a MATLAB simulation to conduct observablity analysis and a script for calibrating parameters using sensor data collected on robot hardware. 

## EKF using Simulation Data
The following script contains a working example 
```matlab
test_ekf.m
```
It simulates the body displacement of the robot in one gait cycle. An EKF estimates robot pose (position and orientation) as well as unknown kinematics parameters. 


## EKF using Hardware Data
We record datasets on a Unitree A1 robot contains proproceptive sensor data, mocap system data and camera images. 

The name of every rosbag indicates the motion pattern of the robot is executing in that rosbag. 

Scripts in folder "hardware_test" runs EKF on these rosbags to estimate the calf link length.

To run the EKF, first load the rosbag. Modify the **bagselect** filepath in 
```matlab
rosbag_process.m
```
So that the correct bag can be loaded. The script also performs some elementary signal processing filters on sensor data.

Then, run the following script
```matlab
test_ekf_hardware.m
```

The estimated calf length will be plotted in a few seconds. The length changes between 0.19-0.21m during walking. 


Comparing to simulation, the hardware estimation is more noisy. This is due to sensor noise and inaccuracy in foot contact detection. Also, as we analysed in the paper, when the joint velocities and body angular velocities are close to zero, kinematic parameters become unobservable. So the estimation will slowly drift when the robot stands still (around 3-6s in datasets).

