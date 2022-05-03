
figure(11)
clf;
plot_start_time = 0;
plot_end_time = param.data_duration;
plot_dt = 0.01;
plot_dim = 1;
subplot(3,1,1)
plot(cali_rho_param);

subplot(3,1,2)
tsout = resample(sensor_data.vel_mocap,plot_start_time:plot_dt:plot_end_time);
tsout_Time = tsout.Time;
tsout_Data = tsout.Data(:,plot_dim);
p1 = plot(tsout_Time,tsout_Data,'LineWidth',3); hold on; 

tsout2 = resample(vel_est_ts1,plot_start_time:plot_dt:plot_end_time);
tsout2_Time = tsout2.Time;
tsout2_Data = tsout2.Data(:,plot_dim);
p2 = plot(tsout2_Time,tsout2_Data,'LineWidth',3); hold on;

tsout3 = resample(vel_est_ts2,plot_start_time:plot_dt:plot_end_time);
tsout3_Time = tsout3.Time;
tsout3_Data = tsout3.Data(:,plot_dim);
p3 = plot(tsout3_Time,tsout3_Data,'LineWidth',3); hold on; 


diff1 = (tsout_Data-tsout2_Data); diff1 = diff1(~isnan(diff1));s1 = sum(diff1'*diff1);
diff2 = (tsout_Data-tsout3_Data); diff2 = diff2(~isnan(diff2));s2 = sum(diff2'*diff2);
legend('Velocity ground truth', strcat('No calibration = ', num2str(s1)), ...
                                strcat('With calibration     = ', num2str(s2)), ...
                       'FontSize', 14,  'Location', 'NorthWest')


subplot(3,1,3)
tsout = resample(sensor_data.pos_mocap,plot_start_time:plot_dt:plot_end_time);
tsout_Time = tsout.Time;
tsout_Data = tsout.Data(:,plot_dim);
p1 = plot(tsout_Time,tsout_Data,'LineWidth',3); hold on; 

tsout2 = resample(pos_est_ts1,plot_start_time:plot_dt:plot_end_time);
tsout2_Time = tsout2.Time;
tsout2_Data = tsout2.Data(:,plot_dim);
p2 = plot(tsout2_Time,tsout2_Data,'LineWidth',3); hold on;

tsout3 = resample(pos_est_ts2,plot_start_time:plot_dt:plot_end_time);
tsout3_Time = tsout3.Time;
tsout3_Data = tsout3.Data(:,plot_dim);
p3 = plot(tsout3_Time,tsout3_Data,'LineWidth',3); hold on; 


diff1 = (tsout_Data-tsout2_Data); diff1 = diff1(~isnan(diff1));s1 = sum(diff1'*diff1);
diff2 = (tsout_Data-tsout3_Data); diff2 = diff2(~isnan(diff2));s2 = sum(diff2'*diff2);
legend('Position ground truth', strcat('No calibration = ', num2str(s1)), ...
                                strcat('With calibration     = ', num2str(s2)), ...
                       'FontSize', 14,  'Location', 'NorthWest')