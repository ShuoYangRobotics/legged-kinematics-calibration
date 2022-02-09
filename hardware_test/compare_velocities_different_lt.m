% 1.1 calculate lo velocity for lc = 0.21
warning('off')

% use default param.lc (should be 0.21)
rho_bias_data = 0.21*ones(size(joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
rho_bias = timeseries(rho_bias_data,joint_ang.Time,'Name',"zero_rho_bias");


lo_v_ts = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_bias, param);

% 1.2 calculate lo velocity for lc = 0.3
warning('off')

% use default lc = 0.3
rho_bias_data2 = 0.3*ones(size(joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
rho_bias2 = timeseries(rho_bias_data2,joint_ang.Time,'Name',"zero_rho_bias");


lo_v_ts2 = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_bias2, param);

%% 1.2 draw 2 lo velocity,compare with ground truth
figure(2);clf

tsout = resample(vel_mocap,8:0.01:13);
p1 = plot(tsout.Time,tsout.Data(:,1),'LineWidth',3);hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')

tsout = resample(lo_v_ts,8:0.01:13);
p2 = plot(tsout.Time,movmean(tsout.Data(:,1),15,1),'LineWidth',2);hold on;
tsout = resample(lo_v_ts2,8:0.01:13);
p3 = plot(tsout.Time,movmean(tsout.Data(:,1),15,1),'LineWidth',2);hold on;
title(['Inaccurate calf length leads to body velocity calculation error'])
xlim([8 13]);
ylim([-0.2 1]);

%% draw foot force flag on the plot
% 1.5 get contact flag from foot_force just for leg one 
contact = 300*contact_estimation(:,1);
% plot(foot_force.Time, tmp);
dtmp = diff(contact);
% plot(foot_force.Time(1:end-1), dtmp);
times = joint_vel.Time;
start_time = times(dtmp>50);
end_time = times(dtmp<-50);
for i=2:min(size(start_time,1),size(end_time,1))
    if (start_time(i)>8 && start_time(i)<13 && end_time(i-1)>8 && end_time(i-1)<13)
        % a = area([start_time(i) end_time(i)],[2 2]);
        a = area([end_time(i-1) start_time(i) ],[2 2]);
        a.FaceAlpha = 0.8;
        a.FaceColor = [0.2 0.6 0.5];
        b = area([end_time(i-1) start_time(i) ],[-2 -2]);
        b.FaceAlpha = 0.8;
        b.FaceColor = [0.2 0.6 0.5];
    end
end
legend([p1,p2,p3],{'Ground truth', 'Calculation using calf length 0.21m', 'Calculation using calf length 0.3m'},'Location','northwest')