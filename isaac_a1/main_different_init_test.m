% assume at this time we have following sensor data time series:
% accel_IMU, gyro_IMU, vel_mocap, 
% foot_force, joint_ang, joint_vel

% we also have vel_mocap which can be used as ground truth

% first add functions in the parent folder
% we especially needs all the kinematicd functions 
addpath('..')
addpath('../unitree_A1/mr')

% run kinematics function generation
% can comment the following two scripts after running them once
run ../kinematics_init_lt_cx
% run ../kinematics_init_cxyz
run ../param_init

%% 1. lc = 0.18
lc1 = 0.18
param.lc = lc1;
param.rho_fix = zeros(param.rho_fix_size,4);
param.rho_fix(:,1) = [param.ox(1);param.oy(1);param.d(1);param.lt; param.lc];
param.rho_fix(:,2) = [param.ox(2);param.oy(2);param.d(2);param.lt; param.lc];
param.rho_fix(:,3) = [param.ox(3);param.oy(3);param.d(3);param.lt; param.lc];
param.rho_fix(:,4) = [param.ox(4);param.oy(4);param.d(4);param.lt; param.lc];

main_simple_ekf_script;

rho_bias1 = rho_bias;

%% 2. lc = 0.25
lc2 = 0.25
param.lc = lc2;
param.rho_fix = zeros(param.rho_fix_size,4);
param.rho_fix(:,1) = [param.ox(1);param.oy(1);param.d(1);param.lt; param.lc];
param.rho_fix(:,2) = [param.ox(2);param.oy(2);param.d(2);param.lt; param.lc];
param.rho_fix(:,3) = [param.ox(3);param.oy(3);param.d(3);param.lt; param.lc];
param.rho_fix(:,4) = [param.ox(4);param.oy(4);param.d(4);param.lt; param.lc];

main_simple_ekf_script;

rho_bias2 = rho_bias;

%% 3. lc = 0.30
lc3 = 0.30
param.lc = lc3;
param.rho_fix = zeros(param.rho_fix_size,4);
param.rho_fix(:,1) = [param.ox(1);param.oy(1);param.d(1);param.lt; param.lc];
param.rho_fix(:,2) = [param.ox(2);param.oy(2);param.d(2);param.lt; param.lc];
param.rho_fix(:,3) = [param.ox(3);param.oy(3);param.d(3);param.lt; param.lc];
param.rho_fix(:,4) = [param.ox(4);param.oy(4);param.d(4);param.lt; param.lc];

main_simple_ekf_script;

rho_bias3 = rho_bias;

%% put all cases together
figure(4); clf
subplot(4,1,1)
plot(rho_bias1.Time, rho_bias1.Data(:,1)+lc1,'LineWidth',3); hold on;
plot(rho_bias2.Time, rho_bias2.Data(:,1)+lc2,'LineWidth',3);
plot(rho_bias3.Time, rho_bias3.Data(:,1)+lc3,'LineWidth',3);
xlabel('Time (s)')
ylabel('calf length (m)')
title('Estimated calf length with different initial values')
legend('lc+\rho for initial lc = 0.18',...
    'lc+\rho for initial lc = 0.25',...
    'lc+\rho for initial lc = 0.30');
subplot(4,1,2)
plot(rho_bias1.Time, rho_bias1.Data(:,2)+lc1,'LineWidth',3); hold on;
plot(rho_bias2.Time, rho_bias2.Data(:,2)+lc2,'LineWidth',3);
plot(rho_bias3.Time, rho_bias3.Data(:,2)+lc3,'LineWidth',3);
xlabel('Time (s)')
ylabel('calf length (m)')
title('Estimated calf length with different initial values')
legend('lc+\rho for initial lc = 0.18',...
    'lc+\rho for initial lc = 0.25',...
    'lc+\rho for initial lc = 0.30');
subplot(4,1,3)
plot(rho_bias1.Time, rho_bias1.Data(:,3)+lc1,'LineWidth',3); hold on;
plot(rho_bias2.Time, rho_bias2.Data(:,3)+lc2,'LineWidth',3);
plot(rho_bias3.Time, rho_bias3.Data(:,3)+lc3,'LineWidth',3);
xlabel('Time (s)')
ylabel('calf length (m)')
title('Estimated calf length with different initial values')
legend('lc+\rho for initial lc = 0.18',...
    'lc+\rho for initial lc = 0.25',...
    'lc+\rho for initial lc = 0.30');