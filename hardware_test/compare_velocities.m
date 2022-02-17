% 1.1 calculate lo velocity
warning('off')

% no bias first
rho_param_data = param.lc*ones(size(joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
rho_param = timeseries(rho_param_data,joint_ang.Time,'Name',"zero_rho_param");


lo_v_ts = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_param, param);

% 1.2 draw lo velocity
figure(1);clf
subplot(2,1,1);
p1 = plot(vel_mocap.Time, vel_mocap.Data(:,1),'LineWidth',3);hold on;
ylim([-0.2 1]);
xlabel('Time (s)')
ylabel('Velocity (m/s)')
% xlim([0 20])
% title(['Ground Truth X-direction body velocity'])
subplot(2,1,2);
p1 = plot(vel_mocap.Time, vel_mocap.Data(:,2),'LineWidth',3);hold on;
ylim([-0.2 1]);
xlabel('Time (s)')
ylabel('Velocity (m/s)')

for i=1:1
    subplot(2,1,1);
    title(['Comparing X directional body velocity. Initial calf length: ' num2str(param.lc)])
    p2 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,1+3*(i-1)),15,1),'LineWidth',2);hold on;
    xlim([6.5 20]);
    subplot(2,1,2);
    title(['Comparing Y directional body velocity. Initial calf length: ' num2str(param.lc)])
    p2 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,2+3*(i-1)),15,1),'LineWidth',2);hold on;
    xlim([6.5 20]);
end