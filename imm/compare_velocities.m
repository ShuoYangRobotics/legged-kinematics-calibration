% 1.1 calculate lo velocity
addpath('../')
warning('off')

% no bias first
rho_bias_data = param.lc*ones(size(joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
rho_bias = timeseries(rho_bias_data,joint_ang.Time,'Name',"zero_rho_bias");


lo_v_ts = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_bias, param);

%% 1.2 draw lo velocity
figure(1);clf

%%
for i=1:4
    subplot(2,1,1);
    title(['Comparing X directional body velocity. Initial calf length: ' num2str(param.lc)])
    p2 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,1+3*(i-1)),1,1),'LineWidth',2);hold on;
    xlim([6.5 20]);
    subplot(2,1,2);
    title(['Comparing Y directional body velocity. Initial calf length: ' num2str(param.lc)])
    p2 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,2+3*(i-1)),1,1),'LineWidth',2);hold on;
    xlim([6.5 20]);
end

%% 
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


subplot(2,1,1);
legend('Leg 1 Velocity estimation', ...
    'Leg 2 Velocity estimation', ...
    'Leg 3 Velocity estimation', ...
    'Leg 4 Velocity estimation', ...
    'Ground Truth')