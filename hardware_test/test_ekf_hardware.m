% assume at this time we have following sensor data time series:
% accel_IMU, gyro_IMU, vel_mocap, 
% foot_force, joint_ang, joint_vel

% we also have vel_mocap which can be used as ground truth

% first add functions in the parent folder
% we especially needs all the kinematicd functions 
addpath('..')
addpath('../mr')

% run kinematics function generation
% can comment the following two scripts after running them once
% run ../kinematics_init_lc
% run ../kinematics_init_cxyz
% run ../param_init

%% 1. lc = 0.18
param.lc_init = 0.18;

imm_kf_together;