% in this script, we assume we get IMU data and a LO velocity output
% we directly fuse these two sensor data to get body velocity and position
% then compare with ground truth
addpath('..')
param.lc_init = 0.20;
% 
imm_kf_together;

% now we have following timeseries
% accel_IMU
% lo_v_ts
% lo_v_ts2
% contact_flags
% vel_mocap
% pos_mocap

[pos_est_ts, vel_est_ts] = imu_lo_fusion(accel_IMU, lo_v_ts, contact_flags);
[pos_est_ts2, vel_est_ts2] = imu_lo_fusion(accel_IMU, lo_v_ts2, contact_flags);


%% compare
plot_start_time = 0;
plot_end_time = 35;
figure(12)
clf
subplot(3,1,1)
tsout = resample(rho_param,plot_start_time:0.005:plot_end_time);
plot(tsout.Time,movmean(tsout.Data(:,[1,2]),30)); hold on; 
plot(tsout.Time,0.21*ones(size(tsout.Time)),'LineWidth',3)
legend('Leg 1', 'Leg 2','0.21m')
xlabel('Time (s)');
ylabel('Calf length (m)');
title('The calibrated calf length during locomotion')
xlim([plot_start_time plot_end_time])
ylim([0 0.3])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
subplot(3,1,2)
tsout = resample(pos_mocap,plot_start_time:0.01:plot_end_time);
plot(tsout.Time,tsout.Data(:,1),'LineWidth',3); hold on; 
tsout = resample(pos_est_ts,plot_start_time:0.01:plot_end_time);
plot(tsout.Time,tsout.Data(1,:),'LineWidth',3); hold on; 
tsout = resample(pos_est_ts2,plot_start_time:0.01:plot_end_time);
plot(tsout.Time,tsout.Data(1,:),'LineWidth',3); hold on; 
legend('Ground truth', 'Estimation using fixed length 0.20m','Estimation using calibrated changing length')
xlabel('Time (s)');
ylabel('X position (m)');
title('Online calibration can reduce long term position estimation drift')
xlim([plot_start_time plot_end_time])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

subplot(3,1,3)
tsout = resample(vel_mocap,plot_start_time:0.01:plot_end_time);
plot(tsout.Time,tsout.Data(:,1),'LineWidth',3); hold on; 
tsout = resample(vel_est_ts,plot_start_time:0.01:plot_end_time);
plot(tsout.Time,tsout.Data(1,:),'LineWidth',3); hold on; 
tsout = resample(vel_est_ts2,plot_start_time:0.01:plot_end_time);
plot(tsout.Time,tsout.Data(1,:),'LineWidth',3); hold on; 
legend('Ground truth', 'Estimation using fixed length 0.20m','Estimation using calibrated changing length')
xlabel('Time (s)');
ylabel('X velocity (m)');
title('Velocity estimation comparison')
xlim([plot_start_time plot_end_time])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

%% save to tikz
% matlab2tikz('position_drift.tikz',...
%             'extraAxisOptions',{'ylabel near ticks','xlabel near ticks',...
%             'scaled ticks=false',...
%             'stack plots=false',...
%             'legend style={at={(0.97,0.97)}, anchor=north east, legend cell align=left, align=left, draw=white!15!black},',...
%             'xtick distance=5',...
%             'ylabel shift = -25pt',...
%             'xlabel shift = -18pt',...
% %             'xtick={0,0.05,0.1}',...
% %             'ytick distance=0.5',...
% %             'minor tick num=1',...
% %             'ylabel style={font=\small\fontfamily{ptm}\selectfont}',...
% %             'xlabel style={font=\small\fontfamily{ptm}\selectfont}',...
% %             'ticklabel style={font=\small\fontfamily{ptm}\selectfont}',...
%             })