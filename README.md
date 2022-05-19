This repo contains code accompanying the following paper:

"Online Kinematic Calibration for Legged Robots",

by Shuo Yang. We proposed a method to let legged robots do online calibration of certain kinematic parameters such as leg length that are hard to measure due to foot deformation. The repo provide a MATLAB simulation to conduct observablity analysis and a script for calibrating parameters using sensor data collected on robot hardware. 

Please notice the code is developed in MATLAB R2022a. And only tested in R2021b and R2022a. 

## EKF using Simulation Data
The following script in folder "sim_data" contains a working example 
```matlab
MAIN_sim.m
```
It simulates the body displacement of the robot in one gait cycle. An EKF estimates robot pose (position and orientation) as well as unknown kinematics parameters. 

This example generates Fig. 4 in the paper.

## EKF using Hardware Data
We record datasets on a Unitree A1 robot contains proproceptive sensor data, mocap system data and camera images. 

First download datasets from 
Dataset link: https://drive.google.com/drive/folders/1s38f7azFWkmGE17V7XUZCtsJWAGyVYnM

Scripts in folder "hardware_data" runs EKF on these rosbags to estimate the calf link length.

Change variable "file_path" to the location of the dataset, then run the script
```matlab
MAIN_kinematic_calibration.m
```

The estimated calf length will be plotted in a few seconds. The length changes between 0.19-0.23m during walking. 

This example generates Fig. 6 in the paper.

