addpath('..')
addpath('../mr')

% run kinematics function generation
% can comment the following two scripts after running them once
% run ../kinematics_init_lt_cx
run ../kinematics_init_cxyz
run ../param_init


% first, calculate LO velocity of each leg

LO_vel_data = zeros(param.num_dof,1);


%% calculate lo velocity
warning('off')

% no bias first
rho_param_data = zeros(size(joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
rho_param = timeseries(rho_param_data,joint_ang.Time,'Name',"zero_rho_param");




%%
figure(3);clf
p1 = plot(vel_mocap.Time, vel_mocap.Data(:,1),'LineWidth',2);hold on;
ylim([-0.2 1]);
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Estimated X directional body velocities using different calf lengths')
for i=1:1
    p2 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,1+3*(i-1)),15,1));hold on;
    xlim([6.5 14.5]);
end
param.lc = 0.18
param.rho_fix = zeros(param.rho_fix_size,4);
param.rho_fix(:,1) = [param.ox(1);param.oy(1);param.d(1);param.lt; param.lc];
param.rho_fix(:,2) = [param.ox(2);param.oy(2);param.d(2);param.lt; param.lc];
param.rho_fix(:,3) = [param.ox(3);param.oy(3);param.d(3);param.lt; param.lc];
param.rho_fix(:,4) = [param.ox(4);param.oy(4);param.d(4);param.lt; param.lc];
lo_v_ts = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_param, param);
for i=1:1
    p3 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,1+3*(i-1)),15,1));hold on;
    xlim([6.5 14.5]);
end
param.lc = 0.30
param.rho_fix = zeros(param.rho_fix_size,4);
param.rho_fix(:,1) = [param.ox(1);param.oy(1);param.d(1);param.lt; param.lc];
param.rho_fix(:,2) = [param.ox(2);param.oy(2);param.d(2);param.lt; param.lc];
param.rho_fix(:,3) = [param.ox(3);param.oy(3);param.d(3);param.lt; param.lc];
param.rho_fix(:,4) = [param.ox(4);param.oy(4);param.d(4);param.lt; param.lc];
lo_v_ts = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_param, param);
for i=1:1
    p4 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,1+3*(i-1)),15,1));hold on;
    xlim([6.5 14.5]);
end
%%
legend([p1 p2 p3 p4],{'ground truth', ...
    'leg 1 LO velocity with calf length 0.25', ...
    'leg 1 LO velocity with calf length 0.18', ...
    'leg 1 LO velocity with calf length 0.30'})