% 1.1 calculate lo velocity
warning('off')

% no bias first
rho_bias_data = param.lc_init*ones(size(joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
rho_bias = timeseries(rho_bias_data,joint_ang.Time,'Name',"zero_rho_bias");


lo_v_ts = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_bias, param);

% 1.2 draw lo velocity
figure(1);clf
subplot(2,1,1);
p1 = plot(vel_mocap.Time, vel_mocap.Data(:,1),'LineWidth',3);hold on;
ylim([-0.2 1]);
xlabel('Time (s)')
ylabel('Velocity (m/s)')
% xlim([0 20])
% title(['Ground Truth X-direction body velocity'])

title(['Comparing X directional body velocity. Initial calf length: ' num2str(param.lc)])
for i=1:1
    subplot(2,1,1);
    p2 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,1+3*(i-1)),15,1),'LineWidth',2);hold on;
    xlim([6.5 20]);
end

% 1.3 run ekf to estimate fk parameters
state_init = [param.lc_init*ones(param.rho_opt_size*param.num_leg,1);
                  zeros(param.rho_opt_size*param.num_leg,1)];

param.simple_ekf_process_position_ratio = 5;
param.simple_ekf_process_velocity_ratio = 0.1;
param.simple_ekf_process_force_ratio = 0;

% estimate along foot_force.Time
[est_state_list,est_state_time] = simple_ekf_estimation(foot_force.Time, state_init, ...
    gyro_IMU, pos_mocap, orient_mocap, vel_mocap, joint_ang, joint_vel, foot_force, param);

rho_bias_data = est_state_list(1:2*param.rho_opt_size*param.num_leg,:)';
rho_bias = timeseries(rho_bias_data,est_state_time,'Name',"rho_bias");

% 1.5 get contact flag from foot_force just for leg one 
ll = movmean(foot_force.Data(:,1),25,1)-80';
no_contact = 1-1./(1+exp(-10*ll));
% contact = movmean(contact,3,1);
contact = 300*(1-no_contact);
% plot(foot_force.Time, tmp);
dtmp = diff(contact);
% plot(foot_force.Time(1:end-1), dtmp);
times = foot_force.Time(1:end-1);
start_time = times(dtmp>50);
end_time = times(dtmp<-50);


% 1.4 draw bias
p3 = subplot(2,1,2);
plot(rho_bias.Time,rho_bias.Data(:,1:param.rho_opt_size)); hold on;
xlim([6.5 20]);
ylim([0 0.35]);
for i=1:min(size(start_time,1),size(end_time,1))
a = area([start_time(i) end_time(i)],[2 2]);
a.FaceAlpha = 0.2;
a.FaceColor = [0.2 0.6 0.5];
end
xlabel('Time (s)')
ylabel('bias (m)')
title('Estimated bias')


for i=1:min(size(start_time,1),size(end_time,1))
subplot(2,1,1);
a = area([start_time(i) end_time(i)],[2 2]);
a.FaceAlpha = 0.2;
a.FaceColor = [0.2 0.6 0.5];
subplot(2,1,2);
a = area([start_time(i) end_time(i)],[350 350]);
a.FaceAlpha = 0.2;
a.FaceColor = [0.2 0.6 0.5];
end


% 1.6 plot on figure 1 of velocity with bias
lo_v_ts2 = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_bias, param);

figure(1)
for i=1:1
    subplot(2,1,1);
    p4 = plot(lo_v_ts2.Time, movmean(lo_v_ts2.Data(:,1+3*(i-1)),15,1),'LineWidth',2);hold on;
    xlim([6.5 20]);
end
subplot(2,1,1);
legend([p1 p2,p4],{'ground truth', 'leg odometry velocity from leg 1 without bias correction',...
    'leg odometry velocity from leg 1 with bias correction'})

subplot(2,1,2);
legend([p3],{'estimated bias from leg 1'})