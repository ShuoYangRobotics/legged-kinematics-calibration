% in this script, we assume we get IMU data and a LO velocity output
% we directly fuse these two sensor data to get body velocity and position
% then compare with ground truth
addpath('..')
param.lc_init = 0.20;
% 
% imm_kf_together;

% now we have following timeseries
% accel_IMU
% lo_v_ts
% lo_v_ts2
% contact_flags
% vel_mocap
% pos_mocap

%%
[pos_est_ts, vel_est_ts] = imu_lo_fusion(accel_IMU, lo_v_ts, contact_flags);
[pos_est_ts2, vel_est_ts2] = imu_lo_fusion(accel_IMU, lo_v_ts2, contact_flags);


%% compare
plot_start_time = 0;
plot_end_time = 35;
figure(12)
clf
set(gcf,'position',[395 826 1197 836])
subplot(3,1,1)
tsout1 = resample(rho_param,plot_start_time:0.005:plot_end_time);
tsout1.Data = movmean(tsout1.Data(:,[1,2]),1);
tsout1_Time = tsout1.Time;
tsout11_Data = tsout1.Data(:,1);
tsout12_Data = tsout1.Data(:,2);
p11 = plot(tsout1_Time,tsout11_Data);hold on; 
p12 = plot(tsout1_Time,tsout12_Data);
p11.XDataSource = 'tsout1_Time';
p11.YDataSource = 'tsout11_Data';
p12.XDataSource = 'tsout1_Time';
p12.YDataSource = 'tsout12_Data';
p1 = plot(tsout1.Time,0.21*ones(size(tsout1.Time)),':','LineWidth',3); 
legend('Leg 1', 'Leg 4', '0.21m')
xlabel('Time (s)');
ylabel('Calf length (m)');
title('Calibrated calf length during locomotion')
xlim([plot_start_time plot_end_time])
ylim([0 0.35])
% ax = gca;
% outerpos = ax.OuterPosition;
% ti = ax.TightInset; 
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax.Position = [left bottom ax_width ax_height];
subplot(3,1,2)
tsout3 = resample(pos_est_ts,plot_start_time:0.01:plot_end_time);
tsout3_Time = tsout3.Time;
tsout3_Data = tsout3.Data(1,:);
p22 = plot(tsout3_Time,tsout3_Data,'LineWidth',3); hold on; 
p22.XDataSource = 'tsout3_Time';
p22.YDataSource = 'tsout3_Data';
tsout4 = resample(pos_est_ts2,plot_start_time:0.01:plot_end_time);
tsout4_Time = tsout4.Time;
tsout4_Data = tsout4.Data(1,:);
p23 = plot(tsout4_Time,tsout4_Data,'LineWidth',3); hold on; 
p23.XDataSource = 'tsout4_Time';
p23.YDataSource = 'tsout4_Data';
tsout2 = resample(pos_mocap,plot_start_time:0.01:plot_end_time);
tsout2_Time = tsout2.Time;
tsout2_Data = tsout2.Data(:,1);
p21 = plot(tsout2_Time,tsout2_Data,'LineWidth',3); hold on; 
p21.XDataSource = 'tsout2_Time';
p21.YDataSource = 'tsout2_Data';
legend('Estimation using fixed length 0.20m','Estimation using calibrated length','Ground truth',  'Location', 'NorthWest')
xlabel('Time (s)');
ylabel('X position (m)');
title('Position estimation using IMU + LO')
xlim([plot_start_time plot_end_time])
ylim([-0.4 3.5])
% ax = gca;
% outerpos = ax.OuterPosition;
% ti = ax.TightInset; 
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax.Position = [left bottom ax_width ax_height];

subplot(3,1,3)
tsout6 = resample(vel_est_ts,plot_start_time:0.01:plot_end_time);
tsout6_Time = tsout6.Time;
tsout6_Data = tsout6.Data(1,:);
p32 = plot(tsout6_Time,tsout6_Data,'LineWidth',3); hold on; 
p32.XDataSource = 'tsout6_Time';
p32.YDataSource = 'tsout6_Data';
tsout7 = resample(vel_est_ts2,plot_start_time:0.01:plot_end_time);
tsout7_Time = tsout7.Time;
tsout7_Data = tsout7.Data(1,:);
p33 = plot(tsout7_Time,tsout7_Data,'LineWidth',3); hold on; 
p33.XDataSource = 'tsout7_Time';
p33.YDataSource = 'tsout7_Data';
tsout5 = resample(vel_mocap,plot_start_time:0.01:plot_end_time);
tsout5_Time = tsout5.Time;
tsout5_Data = tsout5.Data(:,1);
p31 = plot(tsout5_Time,tsout5_Data,'LineWidth',3); hold on; 
p31.XDataSource = 'tsout5_Time';
p31.YDataSource = 'tsout5_Data';
legend('Estimation using fixed length 0.20m','Estimation using calibrated length','Ground truth',  'Location', 'NorthWest')
xlabel('Time (s)');
ylabel('X velocity (m)');
title('Velocity estimation using IMU + LO')
xlim([plot_start_time plot_end_time])
ylim([-1.2 1.4])
% ax = gca;
% outerpos = ax.OuterPosition;
% ti = ax.TightInset; 
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax.Position = [left bottom ax_width ax_height];

%%
remove_nan = isnan(tsout6_Data);
tsout6_Data(remove_nan) = tsout5_Data(remove_nan);
remove_nan = isnan(tsout7_Data);
tsout7_Data(remove_nan) = tsout5_Data(remove_nan);

rmse_fix = (tsout6_Data'-tsout5_Data)'*(tsout6_Data'-tsout5_Data)/length(tsout5_Data)
rmse_calibrate = (tsout7_Data'-tsout5_Data)'*(tsout7_Data'-tsout5_Data)/length(tsout5_Data)
%% save to tikz
matlab2tikz('position_drift2.tikz',...
            'extraAxisOptions',{'ylabel near ticks','xlabel near ticks',...
            'scaled ticks=false',...
            'stack plots=false',...
            'legend style={at={(0.97,0.97)}, anchor=north east, legend cell align=left, align=left, draw=white!15!black},',...
            'xtick distance=5',...
%             'ylabel shift = -25pt',...
%             'xlabel shift = -18pt',...
%             'xtick={0,0.05,0.1}',...
%             'ytick distance=0.5',...
%             'minor tick num=1',...
%             'ylabel style={font=\small\fontfamily{ptm}\selectfont}',...
%             'xlabel style={font=\small\fontfamily{ptm}\selectfont}',...
%             'ticklabel style={font=\small\fontfamily{ptm}\selectfont}',...
            })


%% live update
% DlgH = figure(4);
% H = uicontrol('Style', 'PushButton', ...
%                     'String', 'Break', ...
%                     'Callback', 'delete(gcbf)');
% rosshutdown                
% rosinit           
% node = ros.Node('/matlab_publish');
% elapsed_time = 0;
% time = rostime("now");
% prev_time = time.Sec + 10^-9*time.Nsec;
% curr_time = time.Sec + 10^-9*time.Nsec;
% init_time = time.Sec + 10^-9*time.Nsec;
% TOTAL_TIME = 35;
% figure(12)
% while (1)
%    pause(0.001);
%    time = rostime("now");
%    curr_time  = time.Sec + 10^-9*time.Nsec;
%    dt = curr_time - prev_time;
%    elapsed_time = curr_time - init_time;
%    prev_time = curr_time;
%    disp([dt, elapsed_time]);
%    
%    tsout1 = resample(rho_param,plot_start_time:0.01:elapsed_time);
%    tsout1.Data = movmean(tsout1.Data(:,[1,2]),30);
%    tsout1_Time = tsout1.Time;
%    tsout11_Data = tsout1.Data(:,1);
%    tsout12_Data = tsout1.Data(:,2);
%    
%    
%     tsout2 = resample(pos_mocap,plot_start_time:0.01:elapsed_time);
%     tsout2_Time = tsout2.Time;
%     tsout2_Data = tsout2.Data(:,1);
%     tsout3 = resample(pos_est_ts,plot_start_time:0.01:elapsed_time);
%     tsout3_Time = tsout3.Time;
%     tsout3_Data = tsout3.Data(1,:);
%     tsout4 = resample(pos_est_ts2,plot_start_time:0.01:elapsed_time);
%     tsout4_Time = tsout4.Time;
%     tsout4_Data = tsout4.Data(1,:);
%     
%     
%     tsout5 = resample(vel_mocap,plot_start_time:0.01:elapsed_time);
%     tsout5_Time = tsout5.Time;
%     tsout5_Data = tsout5.Data(:,1);
%     tsout6 = resample(vel_est_ts,plot_start_time:0.01:elapsed_time);
%     tsout6_Time = tsout6.Time;
%     tsout6_Data = tsout6.Data(1,:);
%     tsout7 = resample(vel_est_ts2,plot_start_time:0.01:elapsed_time);
%     tsout7_Time = tsout7.Time;
%     tsout7_Data = tsout7.Data(1,:);
%    refreshdata;
%    
%    if (elapsed_time>TOTAL_TIME)
%        break;
%    end
% end     
% 
% clear('node')