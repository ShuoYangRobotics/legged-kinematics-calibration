% 1.1 calculate lo velocity
warning('off')
param.lc = 0.25

param.rho_fix = zeros(param.rho_fix_size,4);
param.rho_fix(:,1) = [param.ox(1);param.oy(1);param.d(1);param.lt; param.lc];
param.rho_fix(:,2) = [param.ox(2);param.oy(2);param.d(2);param.lt; param.lc];
param.rho_fix(:,3) = [param.ox(3);param.oy(3);param.d(3);param.lt; param.lc];
param.rho_fix(:,4) = [param.ox(4);param.oy(4);param.d(4);param.lt; param.lc];
% no bias first
rho_bias_data = zeros(size(joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
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