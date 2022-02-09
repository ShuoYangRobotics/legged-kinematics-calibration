% assume at this time we have following sensor data time series:
% accel_IMU, gyro_IMU, vel_mocap, 
% foot_force, joint_ang, joint_vel

% we also have pos_mocap which can be used as ground truth

% now our goal is to use above sensor timeseries to understand 
% how much we can detect and estimate the forward kinematics of the robot

% the parameter state:
% rho1 rho2 rho3 rho4 drho1 drho2 drho3 drho3
% process 
% [drho1        [0     0   0   0   1  0   0   0   [ rho1        [ 0
%  drho2         0     0   0   0   0   1  0   0     rho2          0
%  drho3         0     0   0   0   0   0   1  0     rho3          0
%  drho4   =     0     0   0   0   0   0   0   1    rho4   +      0
%  ddrho1       -k     0   0   0   0   0   0   0    drho1        -a*f1
%  ddrho2        0    -k   0   0   0   0   0   0    drho2        -a*f2
%  ddrho3        0     0  -k   0   0   0   0   0    drho3        -a*f3
%  ddrho4]       0     0   0  -k   0   0   0   0]   drho4]       -a*f4

% first add functions in the parent folder
% we especially needs all the kinematicd functions 
addpath('..')
addpath('../unitree_A1/mr')

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
rho_bias_data = zeros(size(joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
rho_bias = timeseries(rho_bias_data,joint_ang.Time,'Name',"zero_rho_bias");


lo_v_ts = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_bias, param);


%%
figure(1);clf
subplot(2,1,1);
p1 = plot(vel_mocap.Time, vel_mocap.Data(:,1),'LineWidth',2);hold on;
ylim([-0.2 1]);
% subplot(4,1,2);
% plot(vel_mocap.Time, vel_mocap.Data(:,2),'LineWidth',2);hold on;
% subplot(4,1,3);
% plot(vel_mocap.Time, vel_mocap.Data(:,3),'LineWidth',2);hold on;
%
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Comparing X directional body velocity. Shade areas are time periods where foot contact sensor reading is larger than 160')
for i=1:1
    subplot(2,1,1);
%     plot(lo_v_ts.Time, lo_v_ts.Data(:,1+3*(i-1)));hold on;
    p2 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,1+3*(i-1)),15,1));hold on;
    xlim([6.5 20]);
    %     subplot(4,1,2);
% %     plot(lo_v_ts.Time, lo_v_ts.Data(:,2+3*(i-1)));hold on;
%     plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,2+3*(i-1)),15,1));hold on;
%     subplot(4,1,3);
% %     plot(lo_v_ts.Time, lo_v_ts.Data(:,3+3*(i-1)));hold on;
%     plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,3+3*(i-1)),15,1));hold on;
end

subplot(2,1,2);
p3 = plot(foot_force.Time, movmean(foot_force.Data(:,1),25,1));hold on;
xlim([6.5 20]);
xlabel('Time (s)')
ylabel('Force (N)')
title('Contact Force Sensor Reading. Shade areas are time periods where foot contact sensor reading is larger than 160')
% ylim([-50 350]);
% plot(foot_force.Time, movmean(foot_force.Data(:,2),15,1));hold on;
% plot(foot_force.Time, movmean(foot_force.Data(:,3),15,1));hold on;
% plot(foot_force.Time, movmean(foot_force.Data(:,4),15,1));hold on;


%get contact flag from foot_force just for leg one 
ll = movmean(foot_force.Data(:,1),25,1)-80'
no_contact = 1-1./(1+exp(-10*ll));
% contact = movmean(contact,3,1);
contact = 300*(1-no_contact);
% plot(foot_force.Time, tmp);
dtmp = diff(contact);
% plot(foot_force.Time(1:end-1), dtmp);
times = foot_force.Time(1:end-1);
start_time = times(dtmp>50);
end_time = times(dtmp<-50);
start_time = uniquetol(start_time,0.01,'DataScale',1);
end_time = uniquetol(end_time,0.01,'DataScale',1);
for i=1:min(size(start_time,1),size(end_time,1))
    if start_time(i)<end_time(i)
        subplot(2,1,1);
        a = area([start_time(i) end_time(i)],[2 2]);
        a.FaceAlpha = 0.2;
        a.FaceColor = [0.2 0.6 0.5];
        subplot(2,1,2);
        a = area([start_time(i) end_time(i)],[350 350]);
        a.FaceAlpha = 0.2;
        a.FaceColor = [0.2 0.6 0.5];
    end
end
subplot(2,1,1);
legend([p1 p2],{'ground truth', 'leg odometry velocity from leg 1'})
subplot(2,1,2);
legend([p3],{'contact force sensor reading from leg 1'})

%%
figure(3);clf
plot(accel_IMU);hold on;
xlim([6.5 20]);
for i=1:min(size(start_time,1),size(end_time,1))
    if start_time(i)<end_time(i)
    a = area([start_time(i) end_time(i)],[40 40]);
    a.FaceAlpha = 0.2;
    a.FaceColor = [0.2 0.6 0.5];
    end
end
%% run ekf to estimate fk parameters
state_init = [0*ones(param.rho_opt_size*param.num_leg,1);
                  zeros(param.rho_opt_size*param.num_leg,1)];

param.simple_ekf_process_position_ratio = 10;
param.simple_ekf_process_velocity_ratio = 10;
param.simple_ekf_process_force_ratio = 0;

% estimate along foot_force.Time
[est_state_list,est_state_time] = simple_ekf_estimation(foot_force.Time, state_init, ...
    gyro_IMU, pos_mocap, orient_mocap, vel_mocap, joint_ang, joint_vel, foot_force, param);

rho_bias_data = est_state_list(1:2*param.rho_opt_size*param.num_leg,:)';
rho_bias = timeseries(rho_bias_data,est_state_time,'Name',"rho_bias");

%% plot state
figure(2);clf
plot(rho_bias.Time,rho_bias.Data(:,1:param.num_leg*param.rho_opt_size)); hold on;
xlim([6.5 20]);
ylim([-0.15 0.05]);
for i=1:size(start_time,1)
a = area([start_time(i) end_time(i)],[2 2]);
a.FaceAlpha = 0.2;
a.FaceColor = [0.2 0.6 0.5];
end
% plot(foot_force.Time, est_state_list(1+param.rho_opt_size*param.num_leg,:)); hold on;

%% plot on figure 1 of velocity with bias


lo_v_ts2 = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_bias, param);

figure(1)
for i=1:1
    subplot(2,1,1);
%     plot(lo_v_ts.Time, lo_v_ts.Data(:,1+3*(i-1)));hold on;
    p4 = plot(lo_v_ts2.Time, movmean(lo_v_ts2.Data(:,1+3*(i-1)),15,1));hold on;
    xlim([6.5 20]);
    %     subplot(4,1,2);
% %     plot(lo_v_ts.Time, lo_v_ts.Data(:,2+3*(i-1)));hold on;
%     plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,2+3*(i-1)),15,1));hold on;
%     subplot(4,1,3);
% %     plot(lo_v_ts.Time, lo_v_ts.Data(:,3+3*(i-1)));hold on;
%     plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,3+3*(i-1)),15,1));hold on;
end
subplot(2,1,1);
legend([p1 p2,p4],{'ground truth', 'leg odometry velocity from leg 1','leg odometry velocity from leg 1 with bias correction'})
subplot(2,1,2);
legend([p3],{'contact force sensor reading from leg 1'})