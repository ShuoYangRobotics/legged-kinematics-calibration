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

%%
lc2 = 0.20
param.lc = lc2;
param.rho_fix = zeros(param.rho_fix_size,4);
param.rho_fix(:,1) = [param.ox(1);param.oy(1);param.d(1);param.lt; param.lc];
param.rho_fix(:,2) = [param.ox(2);param.oy(2);param.d(2);param.lt; param.lc];
param.rho_fix(:,3) = [param.ox(3);param.oy(3);param.d(3);param.lt; param.lc];
param.rho_fix(:,4) = [param.ox(4);param.oy(4);param.d(4);param.lt; param.lc];


% 1.1 calculate lo velocity
warning('off')

% no bias first
rho_bias_data = zeros(size(joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
rho_bias = timeseries(rho_bias_data,joint_ang.Time,'Name',"zero_rho_bias");


lo_v_ts = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_gt, orient_gt,...
    vel_gt, joint_ang, joint_vel,rho_bias, param);

% 1.2 draw lo velocity
figure(1);clf
subplot(3,1,1);
p1 = plot(vel_gt.Time, vel_gt.Data(:,1),'LineWidth',3);hold on;
subplot(3,1,3);
p1 = plot(vel_gt.Time, vel_gt.Data(:,3),'LineWidth',3);hold on;

xlabel('Time (s)')
ylabel('Velocity (m/s)')
% xlim([0 14.5])
% title(['Ground Truth X-direction body velocity'])

title(['Comparing X directional body velocity. Initial calf length: ' num2str(param.lc)])
for i=1:param.num_leg
    subplot(3,1,1);
    p2 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,1+3*(i-1)),15,1),'LineWidth',2);hold on;

    subplot(3,1,3);
    title(['Comparing Z directional body velocity. Initial calf length: ' num2str(param.lc)])
    p4 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,3+3*(i-1)),15,1),'LineWidth',2);hold on;

end

% 1.5 get contact flag from foot_force just for leg one 
ll = movmean(foot_force.Data(:,1),1,1)-90';
no_contact = 1-1./(1+exp(-100*ll));
% contact = movmean(contact,3,1);
contact = 300*(1-no_contact);
dtmp = diff(contact);
% plot(foot_force.Time(1:end-1), dtmp); hold on
times = foot_force.Time(1:end-1);
start_time = times(dtmp>50);
end_time = times(dtmp<-50);
subplot(3,1,2);
plot(foot_force.Time, foot_force.Data(:,1));

for i=1:size(start_time,1)
subplot(3,1,1);
hold on;
a = area([start_time(i) end_time(i)],[1 1]);
a.FaceAlpha = 0.2;
a.FaceColor = [0.2 0.6 0.5];
subplot(3,1,2);
hold on;
a = area([start_time(i) end_time(i)],[250 250]);
a.FaceAlpha = 0.2;
a.FaceColor = [0.2 0.6 0.5];
end

subplot(3,1,3);
plot(gyro_IMU)